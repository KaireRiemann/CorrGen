#include "sdlp_adapter.hpp"

#include <cmath>
#include <stdexcept>
#ifdef CORRGEN_SDLP_SOURCE
#include CORRGEN_SDLP_SOURCE
namespace {
template <int D>
Eigen::VectorXd solve(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                      const Eigen::VectorXd& cost) {
  Eigen::Matrix<double, Eigen::Dynamic, D> matrix = A;
  Eigen::Matrix<double, D, 1> objective = cost, x;
  const double value = math_utils::sdlp::linprog<D>(objective, matrix, b, x);
  if (!std::isfinite(value) || !x.allFinite())
    throw std::runtime_error("SDLP returned infeasible, unbounded, or nonfinite solution");
  return x;
}
}  // namespace
#endif
namespace corrgen {
Eigen::VectorXd solveSdlp(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                          const Eigen::VectorXd& cost) {
#ifdef CORRGEN_SDLP_SOURCE
  switch (A.cols()) {
    case 5:
      return solve<5>(A, b, cost);
    case 20:
      return solve<20>(A, b, cost);
    case 35:
      return solve<35>(A, b, cost);
    default:
      throw std::invalid_argument("SDLP supports degree 0, 3, or 6");
  }
#else
  (void)A;
  (void)b;
  (void)cost;
  throw std::runtime_error("SDLP not built: configure CORRGEN_REAL_PLANNER_DIR");
#endif
}
}  // namespace corrgen
