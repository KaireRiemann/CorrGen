#include "corrgen/path.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace corrgen {
namespace {
std::vector<double> realRoots(std::vector<double> poly) {
  double scale = 0;
  for (double v : poly) scale = std::max(scale, std::abs(v));
  while (poly.size() > 1 && std::abs(poly.back()) <= 1e-13 * scale) poly.pop_back();
  int degree = static_cast<int>(poly.size()) - 1;
  std::vector<double> result;
  if (degree == 1) {
    double u = -poly[0] / poly[1];
    if (u >= -1e-10 && u <= 1 + 1e-10)
      result.push_back(std::clamp(u, 0., 1.));
    return result;
  }
  if (degree == 0)
    return result;
  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
  for (int i = 1; i < degree; ++i) companion(i, i - 1) = 1;
  for (int i = 0; i < degree; ++i) companion(i, degree - 1) = -poly[i] / poly.back();
  Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
  if (solver.info() != Eigen::Success)
    throw std::runtime_error("Polynomial root solve failed");
  for (int i = 0; i < degree; ++i) {
    auto r = solver.eigenvalues()[i];
    if (std::abs(r.imag()) < 1e-7 && r.real() >= -1e-10 && r.real() <= 1 + 1e-10)
      result.push_back(std::clamp(r.real(), 0., 1.));
  }
  return result;
}
}  // namespace
CubicPath::CubicPath(const std::array<Eigen::Vector3d, 4>& p, Eigen::Vector3d up)
    : control_(p), up_(up) {
  for (const auto& v : p)
    if (!v.allFinite())
      throw std::invalid_argument("Nonfinite path");
  if (!up.allFinite() || up.norm() < 1e-10)
    throw std::invalid_argument("Invalid frame axis");
  up_.normalize();
  power_.col(0) = p[0];
  power_.col(1) = 3 * (p[1] - p[0]);
  power_.col(2) = 3 * (p[2] - 2 * p[1] + p[0]);
  power_.col(3) = p[3] - 3 * p[2] + 3 * p[1] - p[0];
  // Minimize squared cross(up,gamma') on [0,1] to reject both stationary
  // references and singular frames. No restriction on x or travel direction.
  std::array<Eigen::Vector3d, 3> v{up_.cross(power_.col(1)), up_.cross(2 * power_.col(2)),
                                   up_.cross(3 * power_.col(3))};
  std::vector<double> norm(5, 0), diff(4, 0);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) norm[i + j] += v[i].dot(v[j]);
  for (int i = 1; i < 5; ++i) diff[i - 1] = i * norm[i];
  auto candidates = realRoots(diff);
  candidates.push_back(0);
  candidates.push_back(1);
  for (double u : candidates)
    if ((v[0] + u * (v[1] + u * v[2])).squaredNorm() < 1e-16)
      throw std::invalid_argument("Reference has a zero tangent or singular frame axis");
}
Eigen::Vector3d CubicPath::derivative(double u, int order) const {
  if (!std::isfinite(u) || u < 0 || u > 1 || order < 1 || order > 2)
    throw std::invalid_argument("Invalid derivative query");
  if (order == 2)
    return 2 * power_.col(2) + 6 * u * power_.col(3);
  return power_.col(1) + 2 * u * power_.col(2) + 3 * u * u * power_.col(3);
}
Eigen::Vector3d CubicPath::position(double u) const {
  if (!std::isfinite(u) || u < 0 || u > 1)
    throw std::invalid_argument("Path u out of range");
  return power_.col(0) + u * (power_.col(1) + u * (power_.col(2) + u * power_.col(3)));
}
Frame CubicPath::frame(double u) const {
  Frame f;
  f.position = position(u);
  Eigen::Vector3d t =
      (power_.col(1) + 2 * u * power_.col(2) + 3 * u * u * power_.col(3)).normalized();
  Eigen::Vector3d n = up_.cross(t).normalized();
  f.rotation.col(0) = t;
  f.rotation.col(1) = n;
  f.rotation.col(2) = t.cross(n);
  return f;
}
std::vector<double> CubicPath::stationaryParameters(const Eigen::Vector3d& point) const {
  if (!point.allFinite())
    throw std::invalid_argument("Nonfinite cloud point");
  // Bernstein convex-hull sign test excludes root-free intervals cheaply.
  constexpr double c3[4] = {1, 3, 3, 1}, c2[3] = {1, 2, 1}, c5[6] = {1, 5, 10, 10, 5, 1};
  std::array<double, 6> bern{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 3; ++j)
      bern[i + j] += c3[i] * c2[j] / c5[i + j] *
                     (control_[i] - point).dot(3 * (control_[j + 1] - control_[j]));
  if (*std::min_element(bern.begin(), bern.end()) > 1e-10 ||
      *std::max_element(bern.begin(), bern.end()) < -1e-10)
    return {};
  auto p = power_;
  p.col(0) -= point;
  std::vector<double> poly(6, 0);
  for (int i = 0; i < 4; ++i)
    for (int j = 1; j < 4; ++j) poly[i + j - 1] += p.col(i).dot(j * power_.col(j));
  return realRoots(poly);
}
std::vector<Occupancy> CubicPath::crossSections(const Eigen::Vector3d& point) const {
  std::vector<Occupancy> result;
  for (double u : stationaryParameters(point)) {
    auto f = frame(u);
    Eigen::Vector3d w = f.rotation.transpose() * (point - f.position);
    if (std::abs(w.x()) > 1e-6)
      throw std::runtime_error("Transverse root residual failed");
    result.push_back({u, w.tail<2>()});
  }
  return result;
}
Occupancy CubicPath::project(const Eigen::Vector3d& point) const {
  auto candidates = stationaryParameters(point);
  candidates.push_back(0);
  candidates.push_back(1);
  double best = 0, distance = (position(0) - point).squaredNorm();
  for (double u : candidates) {
    double d = (position(u) - point).squaredNorm();
    if (d < distance) {
      distance = d;
      best = u;
    }
  }
  auto f = frame(best);
  Eigen::Vector3d local = f.rotation.transpose() * (point - f.position);
  return {best, local.tail<2>()};
}
SplinePath::SplinePath(const std::vector<Eigen::Vector3d>& line) {
  if (line.size() < 2)
    throw std::invalid_argument("Reference requires at least two points");
  for (size_t i = 0; i < line.size(); ++i)
    if (!line[i].allFinite() || (i && (line[i] - line[i - 1]).norm() < 1e-6))
      throw std::invalid_argument("Invalid or duplicate reference points");
  std::vector<Eigen::Vector3d> p;
  p.push_back(2 * line.front() - line[1]);
  p.insert(p.end(), line.begin(), line.end());
  p.push_back(2 * line.back() - line[line.size() - 2]);
  for (size_t i = 0; i + 3 < p.size(); ++i)
    controls_.push_back({(p[i] + 4 * p[i + 1] + p[i + 2]) / 6, (2 * p[i + 1] + p[i + 2]) / 3,
                         (p[i + 1] + 2 * p[i + 2]) / 3, (p[i + 1] + 4 * p[i + 2] + p[i + 3]) / 6});
  // One axis for the entire path avoids frame jumps at piece boundaries.
  Eigen::Vector3d best_axis = Eigen::Vector3d::UnitZ();
  double best = -1;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      for (int z = -1; z <= 1; ++z) {
        Eigen::Vector3d axis(x, y, z);
        if (axis.norm() == 0)
          continue;
        axis.normalize();
        double score = 1;
        for (const auto& c : controls_)
          for (int j = 0; j <= 8; ++j) {
            double u = j / 8.;
            Eigen::Vector3d t = (1 - u) * (1 - u) * (c[1] - c[0]) +
                                2 * (1 - u) * u * (c[2] - c[1]) + u * u * (c[3] - c[2]);
            if (t.norm() < 1e-10) {
              score = 0;
              break;
            }
            score = std::min(score, axis.cross(t.normalized()).squaredNorm());
          }
        if (score > best) {
          best = score;
          best_axis = axis;
        }
      }
  up_ = best_axis;
  for (const auto& c : controls_) segments_.emplace_back(c, best_axis);
}
std::pair<size_t, double> SplinePath::locate(double u) const {
  if (!std::isfinite(u) || u < 0 || u > 1)
    throw std::invalid_argument("Reference u outside [0,1]");
  double t = u * segments_.size();
  size_t i = std::min(static_cast<size_t>(t), segments_.size() - 1);
  return {i, t - i};
}
Eigen::Vector3d SplinePath::position(double u) const {
  auto k = locate(u);
  return segments_[k.first].position(k.second);
}
Eigen::Vector3d SplinePath::derivative(double u, int order) const {
  auto k = locate(u);
  return segments_[k.first].derivative(k.second, order) * std::pow(double(segments_.size()), order);
}
Frame SplinePath::frame(double u) const {
  auto k = locate(u);
  return segments_[k.first].frame(k.second);
}
std::vector<Occupancy> SplinePath::crossSections(const Eigen::Vector3d& point) const {
  std::vector<Occupancy> result;
  for (size_t i = 0; i < segments_.size(); ++i)
    for (auto p : segments_[i].crossSections(point)) {
      p.u = (i + p.u) / segments_.size();
      if (result.empty() || std::abs(result.back().u - p.u) > 1e-10)
        result.push_back(p);
    }
  return result;
}
}  // namespace corrgen
