#include "corrgen/voxel_planner.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>

namespace corrgen {
VoxelPlanner::VoxelPlanner(const std::vector<Eigen::Vector3d>& cloud, Eigen::Vector3d origin,
                           Eigen::Vector3d size, double resolution, double inflation,
                           double clearance_weight)
    : origin_(origin), size_(size), resolution_(resolution), inflation_(inflation) {
  if (!origin.allFinite() || !size.allFinite() || (size.array() <= 0).any() ||
      !std::isfinite(resolution) || resolution <= 0 || !std::isfinite(inflation) || inflation < 0)
    throw std::invalid_argument("Invalid voxel map parameters");
  Eigen::Array3d cells = (size / resolution).array().ceil();
  if (cells.prod() > 4000000)
    throw std::invalid_argument("Voxel map exceeds 4 million cells");
  dims_ = cells.cast<int>();
  occupied_.assign(dims_.prod(), 0);
  std::vector<Eigen::Vector3i> offsets;
  int radius = inflation > 0 ? static_cast<int>(std::ceil(inflation / resolution)) + 1 : 0;
  for (int z = -radius; z <= radius; ++z)
    for (int y = -radius; y <= radius; ++y)
      for (int x = -radius; x <= radius; ++x) {
        // Distance between closed voxel cubes; positive inflation conservatively
        // includes face/edge-adjacent cells touched by the inflated occupied cell.
        Eigen::Vector3d gap =
            (Eigen::Vector3d(x, y, z).cwiseAbs() - Eigen::Vector3d::Ones()).cwiseMax(0.) *
            resolution;
        if ((inflation == 0 && x == 0 && y == 0 && z == 0) ||
            (inflation > 0 && gap.norm() <= inflation + 1e-12))
          offsets.emplace_back(x, y, z);
      }
  // A* visits cell indices directly: boundary erosion must be encoded in the grid too.
  for (int z = 0; z < dims_.z(); ++z)
    for (int y = 0; y < dims_.y(); ++y)
      for (int x = 0; x < dims_.x(); ++x) {
        Eigen::Vector3i c(x, y, z);
        Eigen::Vector3d low = c.cast<double>() * resolution_,
                        high = low + Eigen::Vector3d::Constant(resolution_);
        if ((low.array() < inflation_ - 1e-12).any() ||
            (high.array() > size_.array() - inflation_ + 1e-12).any())
          occupied_[index(c)] = 1;
      }
  for (const auto& p : cloud) {
    if (!p.allFinite())
      throw std::invalid_argument("Nonfinite map point");
    auto c = cell(p);
    if (!valid(c))
      continue;
    for (const auto& offset : offsets) {
      Eigen::Vector3i v = c + offset;
      if (valid(v))
        occupied_[index(v)] = 1;
    }
  }
  if (!std::isfinite(clearance_weight) || clearance_weight < 0)
    throw std::invalid_argument("Invalid clearance preference");
  if (clearance_weight > 0) {
    // Six-neighbour distance is only a routing preference, never a collision certificate.
    std::vector<int> distance(occupied_.size(), -1), queue;
    queue.reserve(occupied_.size());
    for (size_t k = 0; k < occupied_.size(); ++k)
      if (occupied_[k]) {
        distance[k] = 0;
        queue.push_back(k);
      }
    for (size_t i = 0; i < queue.size(); ++i) {
      int k = queue[i];
      auto c = decode(k);
      if (distance[k] >= 16)
        continue;
      for (int axis = 0; axis < 3; ++axis)
        for (int sign : {-1, 1}) {
          auto n = c;
          n[axis] += sign;
          if (!valid(n))
            continue;
          int j = index(n);
          if (distance[j] >= 0)
            continue;
          distance[j] = distance[k] + 1;
          queue.push_back(j);
        }
    }
    traversal_cost_.resize(occupied_.size(), 1);
    for (size_t k = 0; k < distance.size(); ++k)
      if (distance[k] >= 0)
        traversal_cost_[k] =
            1 + clearance_weight * std::exp(-std::max(0., distance[k] - .5) * resolution_ / .5);
  }
}
Eigen::Vector3i VoxelPlanner::cell(const Eigen::Vector3d& p) const {
  return ((p - origin_) / resolution_).array().floor().cast<int>();
}
bool VoxelPlanner::valid(const Eigen::Vector3i& c) const {
  return (c.array() >= 0).all() && (c.array() < dims_.array()).all();
}
int VoxelPlanner::index(const Eigen::Vector3i& c) const {
  return (c.z() * dims_.y() + c.y()) * dims_.x() + c.x();
}
Eigen::Vector3i VoxelPlanner::decode(int k) const {
  return {k % dims_.x(), (k / dims_.x()) % dims_.y(), k / (dims_.x() * dims_.y())};
}
Eigen::Vector3d VoxelPlanner::center(int k) const {
  return origin_ + (decode(k).cast<double>() + Eigen::Vector3d::Constant(.5)) * resolution_;
}
bool VoxelPlanner::free(const Eigen::Vector3d& p) const {
  if (!p.allFinite() || (p.array() < (origin_.array() + inflation_)).any() ||
      (p.array() >= ((origin_ + size_).array() - inflation_)).any())
    return false;
  return !occupied_[index(cell(p))];
}
bool VoxelPlanner::curveFree(const std::array<Eigen::Vector3d, 4>& p, int depth,
                             double clearance) const {
  Eigen::Vector3d low = p[0], high = p[0];
  for (const auto& q : p) {
    low = low.cwiseMin(q);
    high = high.cwiseMax(q);
  }
  if (!free(p.front()) || !free(p.back()))
    return false;
  bool clear = boxFree(low.array() - clearance, high.array() + clearance);
  if (clear)
    return true;
  if (depth <= 0)
    return false;
  Eigen::Vector3d p01 = (p[0] + p[1]) / 2, p12 = (p[1] + p[2]) / 2, p23 = (p[2] + p[3]) / 2;
  Eigen::Vector3d p012 = (p01 + p12) / 2, p123 = (p12 + p23) / 2, mid = (p012 + p123) / 2;
  return curveFree({p[0], p01, p012, mid}, depth - 1, clearance) &&
         curveFree({mid, p123, p23, p[3]}, depth - 1, clearance);
}
bool VoxelPlanner::lineFree(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                            double clearance) const {
  return curveFree({a, a + (b - a) / 3, a + 2 * (b - a) / 3, b}, 12, clearance);
}
double VoxelPlanner::traversalCost(const Eigen::Vector3d& p) const {
  if (traversal_cost_.empty())
    return 1;
  auto c = cell(p);
  return valid(c) ? traversal_cost_[index(c)] : std::numeric_limits<double>::infinity();
}
double VoxelPlanner::lineCost(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
  double length = (b - a).norm();
  if (traversal_cost_.empty())
    return length;
  int count = std::max(1, int(std::ceil(length / (.5 * resolution_))));
  double sum = 0;
  for (int i = 0; i < count; ++i) sum += traversalCost(a + ((i + .5) / count) * (b - a));
  return sum * length / count;
}
std::vector<Eigen::Vector3d> VoxelPlanner::search(const Eigen::Vector3d& start,
                                                  const Eigen::Vector3d& goal) const {
  if (!free(start) || !free(goal))
    throw std::invalid_argument("Selected point is occupied or outside map");
  if ((start - goal).norm() < 1e-5)
    throw std::invalid_argument("Start and goal must differ");
  int s = index(cell(start)), g = index(cell(goal));
  if (s == g)
    return {start, goal};
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
  std::vector<double> cost(occupied_.size(), std::numeric_limits<double>::infinity());
  std::vector<int> parent(occupied_.size(), -1);
  std::vector<unsigned char> closed(occupied_.size(), 0);
  cost[s] = 0;
  queue.push({0, s});
  while (!queue.empty()) {
    int k = queue.top().second;
    queue.pop();
    if (closed[k])
      continue;
    closed[k] = 1;
    if (k == g)
      break;
    auto p = decode(k);
    for (int z = -1; z <= 1; ++z)
      for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x) {
          if (x == 0 && y == 0 && z == 0)
            continue;
          Eigen::Vector3i step(x, y, z), v = p + step;
          if (!valid(v) || occupied_[index(v)])
            continue;
          bool clear = true;
          // All intermediate cells on a face/edge/body diagonal must be free.
          for (int mask = 1; mask < 8; ++mask) {
            Eigen::Vector3i q = p;
            for (int axis = 0; axis < 3; ++axis)
              if (mask & (1 << axis))
                q[axis] += step[axis];
            if (occupied_[index(q)]) {
              clear = false;
              break;
            }
          }
          if (!clear)
            continue;
          int next = index(v);
          double multiplier =
              traversal_cost_.empty() ? 1. : .5 * (traversal_cost_[k] + traversal_cost_[next]);
          double value = cost[k] + step.cast<double>().norm() * resolution_ * multiplier;
          if (value < cost[next]) {
            cost[next] = value;
            parent[next] = k;
            queue.push({value + (center(next) - center(g)).norm(), next});
          }
        }
  }
  if (!closed[g])
    throw std::runtime_error("A* found no connected free path");
  std::vector<Eigen::Vector3d> route;
  for (int k = g; k >= 0; k = parent[k]) route.push_back(center(k));
  std::reverse(route.begin(), route.end());
  // Keep the endpoint-cell centres: directly replacing them can cut a corner.
  if ((route.front() - start).norm() > 1e-6)
    route.insert(route.begin(), start);
  else
    route.front() = start;
  if ((route.back() - goal).norm() > 1e-6)
    route.push_back(goal);
  else
    route.back() = goal;
  return route;
}
SplinePath VoxelPlanner::reference(const std::vector<Eigen::Vector3d>& route,
                                   double clearance) const {
  if (!std::isfinite(clearance) || clearance < 0)
    throw std::invalid_argument("Invalid reference clearance");
  if (route.size() < 2)
    throw std::invalid_argument("Need an A* route");
  // Visibility shortening retains the complete route, including every required turn.
  std::vector<double> prefix(route.size(), 0);
  for (size_t i = 1; i < route.size(); ++i)
    prefix[i] = prefix[i - 1] + lineCost(route[i - 1], route[i]);
  std::vector<Eigen::Vector3d> short_path{route.front()};
  for (size_t i = 0; i + 1 < route.size();) {
    size_t next = route.size() - 1;
    while (next > i + 1 && (!lineFree(route[i], route[next], clearance) ||
                            (!traversal_cost_.empty() &&
                             lineCost(route[i], route[next]) > 1.02 * (prefix[next] - prefix[i]))))
      --next;
    short_path.push_back(route[next]);
    i = next;
  }
  for (int refinement = 0; refinement < 5; ++refinement) {
    double spacing = resolution_ * 2 / std::pow(2., refinement);
    std::vector<Eigen::Vector3d> points{short_path.front()};
    for (size_t i = 1; i < short_path.size(); ++i) {
      int count = std::max(1, int(std::ceil((short_path[i] - short_path[i - 1]).norm() / spacing)));
      for (int k = 1; k <= count; ++k)
        points.push_back(short_path[i - 1] +
                         double(k) / count * (short_path[i] - short_path[i - 1]));
    }
    SplinePath path(points);
    bool valid_curve = true;
    for (const auto& c : path.controls())
      if (!curveFree(c, 12, clearance)) {
        valid_curve = false;
        break;
      }
    if (valid_curve)
      return path;
  }
  throw std::runtime_error(
      "Full A* route exists but its smoothed reference touches occupied voxels");
}

bool VoxelPlanner::boxFree(const Eigen::Vector3d& low, const Eigen::Vector3d& high) const {
  if (!low.allFinite() || !high.allFinite() || (low.array() > high.array()).any() ||
      (low.array() < origin_.array() + inflation_).any() ||
      (high.array() >= (origin_ + size_).array() - inflation_).any())
    return false;
  auto a = cell(low), b = cell(high);
  if (!valid(a) || !valid(b))
    return false;
  for (int z = a.z(); z <= b.z(); ++z)
    for (int y = a.y(); y <= b.y(); ++y)
      for (int x = a.x(); x <= b.x(); ++x)
        if (occupied_[index({x, y, z})])
          return false;
  return true;
}
bool VoxelPlanner::corridorFree(const SplinePath& path, const Corridor& corridor,
                                int max_depth) const {
  std::function<bool(const std::array<Eigen::Vector3d, 4>&, double, double, int)> visit;
  visit = [&](const auto& control, double u0, double u1, int depth) {
    auto lo = control[0], hi = control[0];
    for (const auto& p : control) {
      lo = lo.cwiseMin(p);
      hi = hi.cwiseMax(p);
    }
    double radius = corridor.radiusBound(u0, u1);
    if (std::isfinite(radius) && boxFree(lo.array() - radius, hi.array() + radius))
      return true;
    if (depth <= 0)
      return false;
    auto p01 = ((control[0] + control[1]) * .5).eval(),
         p12 = ((control[1] + control[2]) * .5).eval(),
         p23 = ((control[2] + control[3]) * .5).eval();
    auto p012 = ((p01 + p12) * .5).eval(), p123 = ((p12 + p23) * .5).eval(),
         mid = ((p012 + p123) * .5).eval();
    double um = (u0 + u1) * .5;
    return visit({control[0], p01, p012, mid}, u0, um, depth - 1) &&
           visit({mid, p123, p23, control[3]}, um, u1, depth - 1);
  };
  for (size_t i = 0; i < path.pieces(); ++i)
    if (!visit(path.controls()[i], double(i) / path.pieces(), double(i + 1) / path.pieces(),
               max_depth))
      return false;
  return true;
}
}  // namespace corrgen
