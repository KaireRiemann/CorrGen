#include <chrono>
#include <iostream>
#include <proxsuite/proxqp/dense/dense.hpp>

#include "corrgen/solver.hpp"
namespace corrgen {
LinearSolution solveProxqp(const LinearProblem& p, const Eigen::VectorXd& initial,
                           double budget_ms) {
  (void)budget_ms;  // The process supervisor enforces the wall clock deadline.
  auto begin = std::chrono::steady_clock::now();
  namespace pq = proxsuite::proxqp;
  pq::dense::QP<double> qp(p.cost.size(), 0, p.A.rows());
  qp.settings.eps_abs = 1e-9;
  qp.settings.eps_rel = 0;
  qp.settings.max_iter = 100;
  qp.settings.verbose = false;
  qp.settings.check_duality_gap = true;
  qp.settings.eps_duality_gap_abs = 1e-7;
  qp.settings.eps_duality_gap_rel = 1e-7;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(p.cost.size(), p.cost.size());
  // nullopt uses ProxQP's documented unbounded-side convention.
  qp.init(H, p.cost, proxsuite::nullopt, proxsuite::nullopt, p.A, p.lower, proxsuite::nullopt);
  if (initial.size() == p.cost.size())
    qp.solve(initial, proxsuite::nullopt, proxsuite::nullopt);
  else
    qp.solve();
  LinearSolution result;
  result.x = qp.results.x;
  result.optimal = qp.results.info.status == pq::QPSolverOutput::PROXQP_SOLVED;
  result.status = "PROXQP_" + std::to_string(int(qp.results.info.status));
  result.iterations = qp.results.info.iter;
  result.milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return result;
}
}  // namespace corrgen
