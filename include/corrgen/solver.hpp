#pragma once
#include <Eigen/Core>
#include <string>
namespace corrgen {
// All backends solve min cost.dot(x), A*x >= lower, with free x.
struct LinearProblem {
  Eigen::MatrixXd A;
  Eigen::VectorXd lower, cost;
};
struct LinearSolution {
  Eigen::VectorXd x;
  std::string status;
  bool optimal = false;
  double milliseconds = 0;
  int iterations = 0;
};
LinearSolution solveLinear(const LinearProblem& problem, const std::string& backend,
                           const Eigen::VectorXd& initial, double budget_ms);
bool solverAvailable(const std::string& backend);
}  // namespace corrgen
