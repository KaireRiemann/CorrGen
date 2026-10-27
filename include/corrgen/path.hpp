#pragma once
#include "corrgen/corridor.hpp"
namespace corrgen {
struct Frame {
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
};
// Regular cubic Bezier reference. Its projection enumerates all stationary roots.
class CubicPath {
 public:
  explicit CubicPath(const std::array<Eigen::Vector3d, 4>& control,
                     Eigen::Vector3d up = Eigen::Vector3d::UnitZ());
  Eigen::Vector3d derivative(double u, int order = 1) const;
  Frame frame(double u) const;
  Occupancy project(const Eigen::Vector3d& point) const;
  // All transverse-plane intersections, not just the closest one. Empty means
  // the point is outside the finite swept domain (e.g. beyond a straight end cap).
  std::vector<Occupancy> crossSections(const Eigen::Vector3d& point) const;
  Eigen::Vector3d position(double u) const;

 private:
  std::vector<double> stationaryParameters(const Eigen::Vector3d& point) const;
  Eigen::Matrix<double, 3, 4> power_;
  std::array<Eigen::Vector3d, 4> control_;
  Eigen::Vector3d up_;
};
// Uniform cubic B-spline through a polyline approximation. Endpoints are exact;
// all pieces share one global u and a continuous, nondegenerate frame direction.
class SplinePath {
 public:
  explicit SplinePath(const std::vector<Eigen::Vector3d>& polyline);
  Eigen::Vector3d position(double u) const;
  Eigen::Vector3d derivative(double u, int order = 1) const;
  Frame frame(double u) const;
  std::vector<Occupancy> crossSections(const Eigen::Vector3d& point) const;
  const std::vector<std::array<Eigen::Vector3d, 4>>& controls() const {
    return controls_;
  }
  const Eigen::Vector3d& frameAxis() const {
    return up_;
  }
  size_t pieces() const {
    return segments_.size();
  }

 private:
  std::pair<size_t, double> locate(double u) const;
  Eigen::Vector3d up_;
  std::vector<CubicPath> segments_;
  std::vector<std::array<Eigen::Vector3d, 4>> controls_;
};
}  // namespace corrgen
