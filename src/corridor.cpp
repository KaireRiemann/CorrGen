#include "corrgen/corridor.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace corrgen {
Basis chebyshev(int n, double u) {
  if (n < 0 || n > 30 || !std::isfinite(u) || u < 0 || u > 1)
    throw std::invalid_argument("Chebyshev requires degree 0..30 and u in [0,1]");
  Basis b{Eigen::VectorXd::Zero(n + 1), Eigen::VectorXd::Zero(n + 1), Eigen::VectorXd::Zero(n + 1)};
  b.value[0] = 1;
  const double x = 2 * u - 1;
  if (n) {
    b.value[1] = x;
    b.first[1] = 2;
  }
  for (int k = 2; k <= n; ++k) {
    b.value[k] = 2 * x * b.value[k - 1] - b.value[k - 2];
    b.first[k] = 4 * b.value[k - 1] + 2 * x * b.first[k - 1] - b.first[k - 2];
    b.second[k] = 8 * b.first[k - 1] + 2 * x * b.second[k - 1] - b.second[k - 2];
  }
  return b;
}
Corridor::Corridor(Eigen::MatrixXd coefficients) : coefficients_(std::move(coefficients)) {
  if (coefficients_.rows() != 5 || coefficients_.cols() < 1 || coefficients_.cols() > 31 ||
      !coefficients_.allFinite())
    throw std::invalid_argument("Expected finite 5 x (degree+1) coefficients, degree <=30");
}
Section Corridor::section(double u) const {
  auto b = chebyshev(static_cast<int>(coefficients_.cols()) - 1, u);
  const Eigen::VectorXd v = coefficients_ * b.value, t = coefficients_ * b.first,
                        s = coefficients_ * b.second;
  Section r;
  r.E << v[0], v[2], v[2], v[1];
  r.d = v.tail<2>();
  r.E_u << t[0], t[2], t[2], t[1];
  r.d_u = t.tail<2>();
  r.E_uu << s[0], s[2], s[2], s[1];
  r.d_uu = s.tail<2>();
  return r;
}
Differential Corridor::evaluate(double u, const Eigen::Vector2d& w) const {
  if (!w.allFinite())
    throw std::invalid_argument("Nonfinite transverse position");
  auto s = section(u);
  Differential r;
  r.value = w.dot(s.E * w) + s.d.dot(w) - 1;
  r.gradient[0] = w.dot(s.E_u * w) + s.d_u.dot(w);
  r.gradient.tail<2>() = 2 * s.E * w + s.d;
  r.hessian(0, 0) = w.dot(s.E_uu * w) + s.d_uu.dot(w);
  r.hessian.block<2, 2>(1, 1) = 2 * s.E;
  r.hessian.block<2, 1>(1, 0) = 2 * s.E_u * w + s.d_u;
  r.hessian.block<1, 2>(0, 1) = r.hessian.block<2, 1>(1, 0).transpose();
  return r;
}
Ellipse Corridor::ellipse(double u) const {
  auto s = section(u);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(s.E);
  if (eig.info() != Eigen::Success || eig.eigenvalues().minCoeff() <= 0)
    throw std::runtime_error("Nonpositive ellipse");
  Ellipse e;
  e.center = -0.5 * eig.operatorInverseSqrt() * eig.operatorInverseSqrt() * s.d;
  double scale = 1 + e.center.dot(s.E * e.center);
  e.radii = (scale / eig.eigenvalues().array()).sqrt();
  e.rotation = eig.eigenvectors();
  e.area = std::acos(-1.0) * e.radii.prod();
  return e;
}
namespace {
// Convert Chebyshev T_k(2u-1) to power basis, then degree-n Bernstein basis.
std::vector<long double> bernstein(const Eigen::VectorXd& c) {
  const int n = static_cast<int>(c.size()) - 1;
  std::vector<long double> prev(n + 1), cur(n + 1), power(n + 1), next(n + 1);
  prev[0] = 1;
  power[0] = c[0];
  if (n) {
    cur[0] = -1;
    cur[1] = 2;
  }
  for (int k = 1; k <= n; ++k) {
    for (int j = 0; j <= n; ++j) power[j] += c[k] * cur[j];
    if (k == n)
      break;
    for (int j = 0; j <= n; ++j) next[j] = -2 * cur[j] - prev[j] + (j ? 4 * cur[j - 1] : 0);
    prev = cur;
    cur = next;
  }
  std::vector<long double> b(n + 1);
  for (int i = 0; i <= n; ++i) {
    long double ratio = 1;
    for (int j = 0; j <= i; ++j) {
      if (j)
        ratio *= static_cast<long double>(i - j + 1) / (n - j + 1);
      b[i] += power[j] * ratio;
    }
  }
  return b;
}
bool positive(const std::vector<long double>& b, long double m, int depth) {
  long double low = b[0], high = b[0];
  for (auto x : b) {
    low = std::min(low, x);
    high = std::max(high, x);
  }
  if (low > m)
    return true;
  if (high <= m || depth == 0)
    return false;
  auto t = b;
  std::vector<long double> left(b.size()), right(b.size());
  left[0] = t[0];
  right.back() = t.back();
  for (size_t k = 1; k < b.size(); ++k) {
    for (size_t j = 0; j < b.size() - k; ++j) t[j] = (t[j] + t[j + 1]) / 2;
    left[k] = t[0];
    right[b.size() - k - 1] = t[b.size() - k - 1];
  }
  return positive(left, m, depth - 1) && positive(right, m, depth - 1);
}
}  // namespace
bool Corridor::certifyPositive(double margin, int depth) const {
  if (!std::isfinite(margin) || margin < 0 || depth < 0 || depth > 30)
    throw std::invalid_argument("Invalid certificate options");
  for (int row = 0; row < 2; ++row)
    for (int sign : {-1, 1}) {
      Eigen::VectorXd c = (coefficients_.row(row) + sign * coefficients_.row(2)).transpose();
      if (!positive(bernstein(c), margin, depth))
        return false;
    }
  return true;
}

double Corridor::value(double u, const Eigen::Vector2d& w) const {
  const double weights[5] = {w.x() * w.x(), w.y() * w.y(), 2 * w.x() * w.y(), w.x(), w.y()};
  auto coefficient = [&](int k) {
    double a = 0;
    for (int r = 0; r < 5; ++r) a += weights[r] * coefficients_(r, k);
    return a;
  };
  double b1 = 0, b2 = 0, x = 2 * u - 1;
  for (int k = coefficients_.cols() - 1; k >= 1; --k) {
    double b = 2 * x * b1 - b2 + coefficient(k);
    b2 = b1;
    b1 = b;
  }
  return x * b1 - b2 + coefficient(0) - 1;
}
Corridor Corridor::scaled(double factor) const {
  if (!std::isfinite(factor) || factor <= 0 || factor > 1)
    throw std::invalid_argument("Invalid corridor shrink");
  Eigen::MatrixXd c = coefficients_;
  c.topRows(3) /= factor * factor;
  c.bottomRows(2) /= factor;
  return Corridor(c);
}
namespace {
std::pair<std::vector<long double>, std::vector<long double>> splitAt(std::vector<long double> p,
                                                                      long double t) {
  std::vector<long double> l(p.size()), r(p.size());
  l[0] = p.front();
  r.back() = p.back();
  for (size_t j = 1; j < p.size(); ++j) {
    for (size_t i = 0; i < p.size() - j; ++i) p[i] = (1 - t) * p[i] + t * p[i + 1];
    l[j] = p.front();
    r[p.size() - 1 - j] = p[p.size() - 1 - j];
  }
  return {l, r};
}
std::pair<double, double> rangeOn(const Eigen::VectorXd& coefficients, double begin, double end) {
  auto b = bernstein(coefficients);
  if (end < 1)
    b = splitAt(b, end).first;
  if (begin > 0)
    b = splitAt(b, begin / end).second;
  long double lo = *std::min_element(b.begin(), b.end()),
              hi = *std::max_element(b.begin(), b.end());
  // Guard floating roundoff conservatively; this is not directed-rounding arithmetic.
  double pad = 1e-10 * (1 + std::max(std::abs(lo), std::abs(hi)));
  return {double(lo) - pad, double(hi) + pad};
}
}  // namespace
double Corridor::radiusBound(double begin, double end) const {
  if (begin < 0 || end > 1 || begin >= end)
    throw std::invalid_argument("Invalid bound interval");
  auto a = rangeOn(coefficients_.row(0).transpose(), begin, end),
       b = rangeOn(coefficients_.row(1).transpose(), begin, end);
  auto c = rangeOn(coefficients_.row(2).transpose(), begin, end);
  double cross = std::max(std::abs(c.first), std::abs(c.second));
  double lambda = std::min(a.first - cross, b.first - cross);
  double large = .5 * (a.first + b.first + std::hypot(a.first - b.first, 2 * cross));
  if (a.first > 0 && b.first > 0 && large > 0)
    lambda = std::max(lambda, (a.first * b.first - cross * cross) / large);
  if (lambda <= 0)
    return std::numeric_limits<double>::infinity();
  auto d = rangeOn(coefficients_.row(3).transpose(), begin, end),
       e = rangeOn(coefficients_.row(4).transpose(), begin, end);
  double norm = std::hypot(std::max(std::abs(d.first), std::abs(d.second)),
                           std::max(std::abs(e.first), std::abs(e.second)));
  return (norm + std::sqrt(norm * norm + 4 * lambda)) / (2 * lambda) * (1 + 1e-10);
}
Ellipse Corridor::enclosingEllipse(double begin, double end) const {
  const double radius = radiusBound(begin, end), mid = (begin + end) * .5;
  auto section_mid = section(mid);
  auto result = ellipse(mid);
  if (!std::isfinite(radius)) {
    result.radii.setConstant(std::numeric_limits<double>::infinity());
    return result;
  }
  const double values[5] = {section_mid.E(0, 0), section_mid.E(1, 1), section_mid.E(0, 1),
                            section_mid.d[0], section_mid.d[1]};
  double error[5];
  for (int k = 0; k < 5; ++k) {
    auto bound = rangeOn(coefficients_.row(k).transpose(), begin, end);
    error[k] = std::max(std::abs(bound.first - values[k]), std::abs(bound.second - values[k]));
  }
  // q_mid(w) <= ||E_mid-E(u)|| R² + ||d_mid-d(u)|| R for q_u(w)<=0.
  double excess = (std::max(error[0], error[1]) + error[2]) * radius * radius +
                  std::hypot(error[3], error[4]) * radius;
  double scale = 1 + result.center.dot(section_mid.E * result.center);
  result.radii *= std::sqrt(1 + excess / scale);
  result.area = std::acos(-1.) * result.radii.prod();
  return result;
}
}  // namespace corrgen
