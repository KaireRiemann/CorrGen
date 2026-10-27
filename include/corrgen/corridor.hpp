#pragma once
#include <Eigen/Core>
#include <array>
#include <functional>
#include <string>
#include <vector>

namespace corrgen {
// All derivatives are with respect to normalized path parameter u, not arclength.
struct Basis {
  Eigen::VectorXd value, first, second;
};
Basis chebyshev(int degree, double u);
struct Section {
  Eigen::Matrix2d E, E_u, E_uu;
  Eigen::Vector2d d, d_u, d_uu;
};
struct Differential {
  double value;
  Eigen::Vector3d gradient;  // [u, w1, w2]
  Eigen::Matrix3d hessian;
};
struct Ellipse {
  Eigen::Vector2d center, radii;
  Eigen::Matrix2d rotation;
  double area;
};
class Corridor {
 public:
  explicit Corridor(Eigen::MatrixXd coefficients);
  Section section(double u) const;
  Differential evaluate(double u, const Eigen::Vector2d& w) const;
  Ellipse ellipse(double u) const;
  double value(double u, const Eigen::Vector2d& w) const;
  double radiusBound(double begin, double end) const;
  // All sections on the interval are contained in this transverse ellipse.
  Ellipse enclosingEllipse(double begin, double end) const;
  Corridor scaled(double factor) const;
  // Recursive Bernstein convex-hull certificate over the entire [0,1].
  bool certifyPositive(double margin = 1e-9, int max_depth = 20) const;
  const Eigen::MatrixXd& coefficients() const {
    return coefficients_;
  }

 private:
  Eigen::MatrixXd coefficients_;  // rows a,b,c,d,e; columns Chebyshev degree
};
struct Occupancy {
  double u;
  Eigen::Vector2d w;
};
struct Options {
  std::string solver = "glpk";  // Optional real_planner SDLP: degrees 0, 3, 6.
  int degree = 6;
  int samples = 101;
  double positive_margin = 0.02;
  double exclusion_margin = 1e-5;  // dimensionless q margin, not robot radius
  double wrapper_radius = 3.0;
  int wrapper_angles = 24;
  double feasibility_tolerance = 1e-6;
  bool working_set = true;
  double budget_ms = 1000;
  int max_rounds = 24;
  int batch_size = 128;
};
struct SolveResult {
  Corridor corridor;
  double objective, solve_ms, min_obstacle_residual;
  int constraint_count;
  int working_rows = 0, rounds = 0;
  double backend_ms = 0, verification_ms = 0;
  std::string quality = "FEASIBLE", backend_status;
};
using IncumbentCallback = std::function<void(const SolveResult&)>;
SolveResult generate(const std::vector<Occupancy>& points, const Options& options = {},
                     const IncumbentCallback& on_verified = {});
}  // namespace corrgen
