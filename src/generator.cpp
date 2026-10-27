#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include "corrgen/corridor.hpp"
#include "corrgen/solver.hpp"

namespace corrgen {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
Eigen::VectorXd basis(int degree, double u) {
  Eigen::VectorXd b(degree + 1);
  b[0] = 1;
  if (degree)
    b[1] = 2 * u - 1;
  for (int k = 2; k <= degree; ++k) b[k] = 2 * (2 * u - 1) * b[k - 1] - b[k - 2];
  return b;
}
struct Row {
  double u;
  std::array<double, 5> weights;
  double lower;
};
Eigen::VectorXd rowVector(const Row& row, int degree) {
  auto b = basis(degree, row.u);
  Eigen::VectorXd v(5 * b.size());
  for (int k = 0; k < 5; ++k) v.segment(k * b.size(), b.size()) = row.weights[k] * b;
  return v;
}
}  // namespace
SolveResult generate(const std::vector<Occupancy>& points, const Options& o,
                     const IncumbentCallback& callback) {
  if (!solverAvailable(o.solver))
    throw std::invalid_argument("Backend not available: " + o.solver);
  if (o.solver == "sdlp" && o.degree != 0 && o.degree != 3 && o.degree != 6)
    throw std::invalid_argument("SDLP supports degree 0, 3, 6");
  if (o.degree < 0 || o.degree > 20 || o.samples < 4 * (o.degree + 1) || o.samples < 2 ||
      o.wrapper_angles < 8 || !std::isfinite(o.positive_margin) || o.positive_margin <= 0 ||
      !std::isfinite(o.wrapper_radius) || o.wrapper_radius <= 0 ||
      !std::isfinite(o.exclusion_margin) || o.exclusion_margin <= 0 ||
      !std::isfinite(o.feasibility_tolerance) || o.feasibility_tolerance <= 0 ||
      !std::isfinite(o.budget_ms) || o.budget_ms <= 0 || o.max_rounds < 1 || o.batch_size < 1)
    throw std::invalid_argument("Invalid generator options");
  const auto begin = Clock::now();
  const int n = o.degree + 1, dim = 5 * n;
  double min_distance = o.wrapper_radius;
  for (const auto& p : points) {
    if (!std::isfinite(p.u) || p.u < 0 || p.u > 1 || !p.w.allFinite())
      throw std::invalid_argument("Invalid occupancy");
    min_distance = std::min(min_distance, p.w.norm());
  }
  if (min_distance < 1e-10)
    throw std::runtime_error("Obstacle intersects reference; no finite point-model seed");
  std::vector<Row> fixed;
  Eigen::VectorXd cost = Eigen::VectorXd::Zero(dim);
  for (int i = 0; i < o.samples; ++i) {
    double u = double(i) / (o.samples - 1);
    auto b = basis(o.degree, u);
    cost.head(n) += b;
    cost.segment(n, n) += b;
    for (int sign : {-1, 1}) {
      fixed.push_back({u, {1, 0, double(sign), 0, 0}, o.positive_margin});
      fixed.push_back({u, {0, 1, double(sign), 0, 0}, o.positive_margin});
    }
    for (int j = 0; j < o.wrapper_angles; ++j) {
      double a = 2 * std::acos(-1.) * j / o.wrapper_angles, y = o.wrapper_radius * std::cos(a),
             z = o.wrapper_radius * std::sin(a);
      fixed.push_back({u, {y * y, z * z, 2 * y * z, y, z}, 1 + o.exclusion_margin});
    }
  }
  Eigen::MatrixXd base(fixed.size(), dim);
  Eigen::VectorXd lower(fixed.size());
  for (size_t i = 0; i < fixed.size(); ++i) {
    base.row(i) = rowVector(fixed[i], o.degree);
    lower[i] = fixed[i].lower;
  }
  auto makeCorridor = [&](const Eigen::VectorXd& x) {
    Eigen::MatrixXd c(5, n);
    for (int r = 0; r < 5; ++r) c.row(r) = x.segment(r * n, n).transpose();
    return Corridor(c);
  };
  Eigen::VectorXd initial = Eigen::VectorXd::Zero(dim);
  initial[0] = initial[n] =
      1.01 * std::max(o.positive_margin, (1 + o.exclusion_margin) / (min_distance * min_distance));
  SolveResult best{makeCorridor(initial), cost.dot(initial), 0,
                   std::numeric_limits<double>::infinity(), int(points.size() + fixed.size())};
  auto verify = [&](const Corridor& c, const Eigen::VectorXd& x, double& min_q) {
    if (!x.allFinite() || (base * x - lower).minCoeff() < -o.feasibility_tolerance ||
        !c.certifyPositive())
      return false;
    min_q = std::numeric_limits<double>::infinity();
    for (const auto& p : points) min_q = std::min(min_q, c.value(p.u, p.w));
    return min_q >= o.exclusion_margin - o.feasibility_tolerance;
  };
  if (!verify(best.corridor, initial, best.min_obstacle_residual))
    throw std::runtime_error("Analytic seed verification failed");
  best.quality = "FEASIBLE_SEED";
  best.solve_ms = elapsed(begin);
  if (callback)
    callback(best);
  std::vector<size_t> active;
  std::vector<unsigned char> included(points.size(), 0);
  auto add = [&](size_t i) {
    if (!included[i]) {
      included[i] = 1;
      active.push_back(i);
    }
  };
  if (!o.working_set) {
    for (size_t i = 0; i < points.size(); ++i) add(i);
  } else {
    // Select nearest point in each parameter/direction bucket; never discard the rest.
    const int bins = 16, angles = 8;
    std::vector<size_t> ids(bins * angles, points.size());
    std::vector<double> distances(bins * angles, std::numeric_limits<double>::infinity());
    for (size_t i = 0; i < points.size(); ++i) {
      const auto& p = points[i];
      int u = std::min(bins - 1, int(p.u * bins));
      int a = std::min(angles - 1, int((std::atan2(p.w.y(), p.w.x()) + std::acos(-1.)) /
                                       (2 * std::acos(-1.)) * angles));
      int b = u * angles + a;
      if (p.w.squaredNorm() < distances[b]) {
        distances[b] = p.w.squaredNorm();
        ids[b] = i;
      }
    }
    for (auto id : ids)
      if (id < points.size())
        add(id);
  }
  Eigen::VectorXd warm = initial;
  double backend_ms = 0, verification_ms = 0;
  for (int round = 0; round < o.max_rounds && elapsed(begin) < o.budget_ms; ++round) {
    LinearProblem lp;
    lp.A.resize(fixed.size() + active.size(), dim);
    lp.lower.resize(lp.A.rows());
    lp.cost = cost;
    lp.A.topRows(fixed.size()) = base;
    lp.lower.head(fixed.size()) = lower;
    for (size_t k = 0; k < active.size(); ++k) {
      const auto& p = points[active[k]];
      double y = p.w.x(), z = p.w.y();
      lp.A.row(fixed.size() + k) =
          rowVector({p.u, {y * y, z * z, 2 * y * z, y, z}, 1 + o.exclusion_margin}, o.degree);
      lp.lower[fixed.size() + k] = 1 + o.exclusion_margin;
    }
    // Positive row scaling preserves the feasible set; final tests use original units.
    for (int i = 0; i < lp.A.rows(); ++i) {
      double scale = std::max(1., lp.A.row(i).norm());
      lp.A.row(i) /= scale;
      lp.lower[i] /= scale;
    }
    double remaining = o.budget_ms - elapsed(begin);
    if (remaining <= 0)
      break;
    auto solution = solveLinear(lp, o.solver, warm, remaining);
    backend_ms += solution.milliseconds;
    best.backend_status = solution.status;
    best.working_rows = lp.A.rows();
    best.rounds = round + 1;
    if (solution.x.size() != dim || !solution.x.allFinite()) {
      best.quality = "FEASIBLE_BACKEND_LIMIT";
      break;
    }
    auto candidate = makeCorridor(solution.x);
    auto check_begin = Clock::now();
    std::vector<std::pair<double, size_t>> violations;
    double min_q = std::numeric_limits<double>::infinity();
    double blend = 1.;
    for (size_t i = 0; i < points.size(); ++i) {
      double q = candidate.value(points[i].u, points[i].w);
      min_q = std::min(min_q, q);
      const double target = o.exclusion_margin + o.feasibility_tolerance;
      if (q < target) {
        double seed = initial[0] * points[i].w.squaredNorm() - 1;
        blend = std::min(blend, (seed - target) / (seed - q));
      }
      if (q < o.exclusion_margin - o.feasibility_tolerance && !included[i])
        violations.emplace_back(o.exclusion_margin - q, i);
    }
    bool fixed_ok = (base * solution.x - lower).minCoeff() >= -o.feasibility_tolerance;
    bool all_ok = fixed_ok && min_q >= o.exclusion_margin - o.feasibility_tolerance &&
                  candidate.certifyPositive();
    verification_ms += elapsed(check_begin);
    if (all_ok) {
      double objective = cost.dot(solution.x);
      if (objective <= best.objective) {
        best.corridor = std::move(candidate);
        best.objective = objective;
        best.min_obstacle_residual = min_q;
        best.quality = solution.optimal ? "OPTIMAL_SOLVER_TOLERANCE" : "FEASIBLE";
        best.solve_ms = elapsed(begin);
        best.backend_ms = backend_ms;
        best.verification_ms = verification_ms;
        if (callback)
          callback(best);
      }
      break;
    }
    // Convexly repair toward the known feasible seed; retain this full-constraint
    // incumbent while the restricted LP still omits violated obstacles.
    Eigen::VectorXd candidate_fixed = base * solution.x, seed_fixed = base * initial;
    for (int i = 0; i < lower.size(); ++i) {
      double target = lower[i] + o.feasibility_tolerance;
      if (candidate_fixed[i] < target)
        blend = std::min(blend, (seed_fixed[i] - target) / (seed_fixed[i] - candidate_fixed[i]));
    }
    blend = std::max(0., std::min(1., blend)) * (1 - 1e-8);
    for (int attempt = 0; attempt < 12 && blend > 0; ++attempt) {
      Eigen::VectorXd repaired = initial + blend * (solution.x - initial);
      auto c = makeCorridor(repaired);
      double q;
      if (verify(c, repaired, q)) {
        double objective = cost.dot(repaired);
        if (objective < best.objective * (1 - 1e-8)) {
          best.corridor = std::move(c);
          best.objective = objective;
          best.min_obstacle_residual = q;
          best.quality = "FEASIBLE_REPAIRED";
          best.solve_ms = elapsed(begin);
          best.backend_ms = backend_ms;
          best.verification_ms = verification_ms;
          if (callback)
            callback(best);
        }
        break;
      }
      blend *= .5;
    }
    if (violations.empty()) {
      best.quality = "FEASIBLE_NUMERICAL_LIMIT";
      break;
    }
    // Batch the worst violation in each parameter bucket before filling remaining slots.
    std::sort(violations.begin(), violations.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });
    int added = 0;
    std::vector<unsigned char> used(32, 0);
    for (const auto& v : violations) {
      int bin = std::min(31, int(points[v.second].u * 32));
      if (!used[bin] && added < o.batch_size) {
        add(v.second);
        used[bin] = 1;
        ++added;
      }
    }
    for (const auto& v : violations)
      if (!included[v.second] && added < o.batch_size) {
        add(v.second);
        ++added;
      }
    warm = solution.x;
  }
  if (elapsed(begin) >= o.budget_ms && best.quality != "OPTIMAL_SOLVER_TOLERANCE")
    best.quality = "FEASIBLE_BUDGET";
  best.solve_ms = elapsed(begin);
  best.backend_ms = backend_ms;
  best.verification_ms = verification_ms;
  return best;
}
}  // namespace corrgen
