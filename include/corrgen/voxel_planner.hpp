#pragma once
#include "corrgen/path.hpp"

namespace corrgen {
// Point cloud -> occupied voxel cells; optional inflation; 26-neighbour A*.
// This planner supports arbitrary travel directions and different endpoint heights.
struct CorridorCheck {
  bool verified = true;
  bool unresolved = false;
  double unresolved_u = -1;
  double unresolved_error = 0, unresolved_q = 0;
  std::vector<Occupancy> witnesses;
  size_t boxes_tested = 0;
};
class VoxelPlanner {
 public:
  VoxelPlanner(const std::vector<Eigen::Vector3d>& cloud, Eigen::Vector3d origin,
               Eigen::Vector3d size, double resolution, double inflation,
               double clearance_weight = 0);
  // Legacy isotropic enclosure for comparisons; ROS uses checkCorridor below.
  bool corridorFree(const SplinePath& path, const Corridor& corridor, int max_depth = 12) const;
  // Directional ellipse support separates the whole interval from each occupied box.
  CorridorCheck checkCorridor(const SplinePath& path, const Corridor& corridor, int max_depth = 12,
                              size_t max_witnesses = 128, bool sample_only = false) const;
  bool boxFree(const Eigen::Vector3d& low, const Eigen::Vector3d& high) const;
  bool free(const Eigen::Vector3d& point) const;
  std::vector<Eigen::Vector3d> search(const Eigen::Vector3d& start,
                                      const Eigen::Vector3d& goal) const;
  SplinePath reference(const std::vector<Eigen::Vector3d>& route, double clearance = 0) const;
  bool curveFree(const std::array<Eigen::Vector3d, 4>& controls, int depth = 12,
                 double clearance = 0) const;

 private:
  Eigen::Vector3i cell(const Eigen::Vector3d& point) const;
  bool valid(const Eigen::Vector3i& cell) const;
  int index(const Eigen::Vector3i& cell) const;
  Eigen::Vector3i decode(int index) const;
  Eigen::Vector3d center(int index) const;
  bool lineFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b, double clearance = 0) const;
  Eigen::Vector3d origin_, size_;
  Eigen::Vector3i dims_;
  double resolution_, inflation_;
  double traversalCost(const Eigen::Vector3d& point) const;
  double lineCost(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
  std::vector<float> traversal_cost_;
  std::vector<unsigned char> occupied_;
};
}  // namespace corrgen
