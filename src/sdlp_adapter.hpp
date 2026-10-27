#pragma once
#include <Eigen/Core>
namespace corrgen {
Eigen::VectorXd solveSdlp(const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                          const Eigen::VectorXd& cost);
}
