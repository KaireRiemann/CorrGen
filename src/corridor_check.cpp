#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>

#include "corrgen/voxel_planner.hpp"

namespace corrgen {
namespace {
using V3 = Eigen::Vector3d;
using V2 = Eigen::Vector2d;
using Box = std::array<V3, 8>;
Box corners(const V3& lo, const V3& hi) {
  Box v;
  for (int i = 0; i < 8; ++i)
    for (int j = 0; j < 3; ++j) v[i][j] = (i & (1 << j)) ? hi[j] : lo[j];
  return v;
}
struct Slice {
  bool empty = true;
  V2 closest = V2::Zero();
  double q = 0;
};
Slice sliceMinimum(const Box& box, const Frame& f, const Section& s, bool project_box = false) {
  std::vector<V2> vertices;
  const V3 normal = f.rotation.col(0);
  auto add = [&](const V3& p) {
    V2 w = f.rotation.rightCols<2>().transpose() * (p - f.position);
    for (const auto& v : vertices)
      if ((v - w).squaredNorm() < 1e-20)
        return;
    vertices.push_back(w);
  };
  for (int i = 0; i < 8; ++i) {
    double a = normal.dot(box[i] - f.position);
    if (std::abs(a) < 1e-10)
      add(box[i]);
    for (int j = 0; j < 3; ++j)
      if (!(i & (1 << j))) {
        int k = i | (1 << j);
        double b = normal.dot(box[k] - f.position);
        if (a * b < 0)
          add(box[i] + (a / (a - b)) * (box[k] - box[i]));
      }
  }
  if (project_box) {
    vertices.clear();
    for (const auto& p : box) add(p);
  }
  Slice result;
  if (vertices.empty())
    return result;
  result.empty = false;
  result.q = std::numeric_limits<double>::infinity();
  std::sort(vertices.begin(), vertices.end(), [](const V2& a, const V2& b) {
    return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
  });
  auto cross = [](const V2& a, const V2& b, const V2& c) {
    V2 x = b - a, y = c - a;
    return x.x() * y.y() - x.y() * y.x();
  };
  if (vertices.size() > 2) {
    std::vector<V2> hull;
    for (const auto& p : vertices) {
      while (hull.size() > 1 && cross(hull[hull.size() - 2], hull.back(), p) <= 1e-14)
        hull.pop_back();
      hull.push_back(p);
    }
    size_t lower_size = hull.size();
    for (int i = int(vertices.size()) - 2; i >= 0; --i) {
      while (hull.size() > lower_size &&
             cross(hull[hull.size() - 2], hull.back(), vertices[i]) <= 1e-14)
        hull.pop_back();
      hull.push_back(vertices[i]);
    }
    hull.pop_back();
    vertices = std::move(hull);
  }
  auto consider = [&](const V2& w) {
    double q = w.dot(s.E * w) + s.d.dot(w) - 1;
    if (q < result.q) {
      result.q = q;
      result.closest = w;
    }
  };
  V2 centre = -.5 * s.E.inverse() * s.d;
  bool inside = vertices.size() >= 3;
  for (size_t i = 0; i < vertices.size(); ++i) {
    V2 a = vertices[i], edge = vertices[(i + 1) % vertices.size()] - a, delta = centre - a;
    if (edge.x() * delta.y() - edge.y() * delta.x() < -1e-12)
      inside = false;
    double denominator = 2 * edge.dot(s.E * edge);
    double t = denominator > 1e-20 ? -edge.dot(2 * s.E * a + s.d) / denominator : 0;
    consider(a + std::clamp(t, 0., 1.) * edge);
  }
  if (inside)
    consider(centre);
  return result;
}
}  // namespace
CorridorCheck VoxelPlanner::checkCorridor(const SplinePath& path, const Corridor& corridor,
                                          int max_depth, size_t max_witnesses,
                                          bool sample_only) const {
  if (max_depth < 0 || max_depth > 24 || max_witnesses == 0)
    throw std::invalid_argument("Invalid continuous check budget");
  CorridorCheck result;
  constexpr double guard = 1e-9;
  std::function<bool(const std::array<V3, 4>&, double, double, int)> visit;
  visit = [&](const auto& control, double u0, double u1, int depth) {
    const double mid = .5 * (u0 + u1);
    const Frame f = path.frame(mid);
    const auto e = sample_only ? corridor.ellipse(mid) : corridor.enclosingEllipse(u0, u1);
    const auto actual = corridor.ellipse(mid);
    const Section section = corridor.section(mid);
    const Eigen::Matrix<double, 3, 2> basis = f.rotation.rightCols<2>();
    const V3 centre = f.position + basis * e.center;
    const Eigen::Matrix<double, 3, 2> axes = basis * e.rotation * e.radii.asDiagonal();
    const Eigen::Matrix<double, 3, 2> exact_axes =
        basis * actual.rotation * actual.radii.asDiagonal();
    const V3 exact_centre = f.position + basis * actual.center;
    double position_error = 0;
    for (const auto& c : control)
      position_error = std::max(position_error, (c - f.position).norm());
    const V3 v0 = .75 * (control[1] - control[0]) + 1.5 * (control[2] - control[1]) +
                  .75 * (control[3] - control[2]);
    double velocity_error = 0;
    for (int j = 0; j < 3; ++j)
      velocity_error = std::max(velocity_error, (3 * (control[j + 1] - control[j]) - v0).norm());
    const double tangent_error = std::min(2., 2 * velocity_error / std::max(v0.norm(), 1e-30));
    const double normal_error =
        std::min(2., 2 * path.frameAxis().norm() * tangent_error /
                         std::max(path.frameAxis().cross(f.rotation.col(0)).norm(), 1e-30));
    const double frame_error = std::hypot(normal_error, std::min(2., normal_error + tangent_error));
    const double radius =
        sample_only ? e.center.norm() + e.radii.maxCoeff() : corridor.radiusBound(u0, u1);
    const double error = sample_only ? guard : position_error + frame_error * radius + guard;
    auto support = [&](const V3& n) {
      double position_support = n.dot(f.position);
      if (!sample_only) {
        position_support = -std::numeric_limits<double>::infinity();
        for (const auto& c : control) position_support = std::max(position_support, n.dot(c));
      }
      double frame_pad = sample_only ? guard : frame_error * radius + guard;
      return position_support + n.dot(basis * e.center) + (axes.transpose() * n).norm() +
             frame_pad * n.norm();
    };
    auto exactSupport = [&](const V3& n) {
      return n.dot(exact_centre) + (exact_axes.transpose() * n).norm();
    };
    bool resolved = std::isfinite(error) && axes.allFinite();
    bool collision = false;
    double ambiguous_q = -999;
    std::optional<Box> ambiguous_box;
    auto witness = [&](const V2& w) {
      if (result.witnesses.size() < max_witnesses)
        result.witnesses.push_back({mid, w});
      collision = true;
    };
    if (resolved) {
      // Six domain planes: use actual ellipse support, not its largest semiaxis.
      for (int axis = 0; axis < 3; ++axis)
        for (int sign : {-1, 1}) {
          V3 n = V3::Zero();
          n[axis] = sign;
          double boundary =
              sign > 0 ? origin_[axis] + size_[axis] - inflation_ : -origin_[axis] - inflation_;
          if (support(n) < boundary - guard)
            continue;
          resolved = false;
          if (exactSupport(n) >= boundary - guard) {
            V2 normal = basis.transpose() * n;
            V2 inverse = section.E.inverse() * normal;
            double denominator = normal.dot(inverse);
            if (denominator > 1e-20)
              witness(actual.center +
                      inverse * ((boundary - n.dot(f.position) - normal.dot(actual.center)) /
                                 denominator));
          }
        }
      V3 lo, hi;
      for (int axis = 0; axis < 3; ++axis) {
        V3 n = V3::Unit(axis);
        hi[axis] = support(n);
        lo[axis] = -support(-n);
      }
      Eigen::Vector3i a = cell(lo).cwiseMax(Eigen::Vector3i::Zero());
      Eigen::Vector3i b = cell(hi).cwiseMin(dims_ - Eigen::Vector3i::Ones());
      std::vector<std::pair<double, int>> nearby;
      for (int z = a.z(); z <= b.z(); ++z)
        for (int y = a.y(); y <= b.y(); ++y)
          for (int x = a.x(); x <= b.x(); ++x) {
            int k = index({x, y, z});
            if (occupied_[k])
              nearby.emplace_back((center(k) - f.position).squaredNorm(), k);
          }
      if (sample_only)
        std::sort(nearby.begin(), nearby.end());
      for (const auto& item : nearby) {
        if (collision)
          break;
        Eigen::Vector3i c = decode(item.second);
        ++result.boxes_tested;
        V3 low = origin_ + c.cast<double>() * resolution_;
        const Box box = corners(low, low + V3::Constant(resolution_));
        auto separates = [&](const V3& n) {
          double minimum = std::numeric_limits<double>::infinity();
          for (const auto& v : box) minimum = std::min(minimum, n.dot(v));
          return support(n) < minimum - guard * n.norm();
        };
        if (separates(f.rotation.col(0)) || separates(-f.rotation.col(0)))
          continue;
        auto slice = sliceMinimum(box, f, section);
        if (!slice.empty && slice.q <= 1e-7) {
          witness(slice.closest);
          break;
        }
        bool separated = false;
        auto projected = sliceMinimum(box, f, section, true);
        if (!projected.empty && projected.q > 0) {
          V3 n = basis * (2 * section.E * projected.closest + section.d);
          if (n.norm() > 1e-20)
            separated = separates(n.normalized());
        }

        if (!separated && !slice.empty) {
          V3 g = basis * (2 * section.E * slice.closest + section.d);
          double length = g.norm();
          if (length > 1e-20) {
            g /= length;
            double h = exactSupport(g);
            double gap = .25 * std::max(0., g.dot(f.position + basis * slice.closest) - h);
            double lower = -std::numeric_limits<double>::infinity(), upper = -lower;
            bool possible = true;
            for (const auto& v : box) {
              double d = f.rotation.col(0).dot(v - f.position), rhs = h + gap - g.dot(v);
              if (d > 1e-12)
                lower = std::max(lower, rhs / d);
              else if (d < -1e-12)
                upper = std::min(upper, rhs / d);
              else if (rhs > 1e-9)
                possible = false;
            }
            if (possible && lower <= upper) {
              double alpha = std::max(lower, std::min(upper, 0.));
              V3 n = g + alpha * f.rotation.col(0);
              if (n.allFinite())
                separated = separates(n.normalized());
            }
          }
        }
        if (!separated && !sample_only && slice.empty && depth <= 3) {
          // Alternating projections only propose a separating direction;
          // acceptance still uses the interval support inequality above.
          const auto rotation = (basis * actual.rotation).eval();
          V3 bpoint = exact_centre.cwiseMax(low).cwiseMin(low + V3::Constant(resolution_));
          for (int iteration = 0; iteration < 32 && !separated; ++iteration) {
            V2 z = rotation.transpose() * (bpoint - exact_centre), projected = z;
            if ((z.array() / actual.radii.array()).square().sum() > 1) {
              double lower = 0, upper = std::max(1., z.norm() * actual.radii.maxCoeff());
              auto length = [&](double lambda) {
                return (actual.radii.array() * z.array() / (actual.radii.array().square() + lambda))
                    .square()
                    .sum();
              };
              while (length(upper) > 1) upper *= 2;
              for (int j = 0; j < 40; ++j) {
                double mid = .5 * (lower + upper);
                if (length(mid) > 1)
                  lower = mid;
                else
                  upper = mid;
              }
              projected = (actual.radii.array().square() * z.array() /
                           (actual.radii.array().square() + upper))
                              .matrix();
            }
            V3 apoint = exact_centre + rotation * projected;
            bpoint = apoint.cwiseMax(low).cwiseMin(low + V3::Constant(resolution_));
            V3 n = bpoint - apoint;
            if (n.norm() < 1e-12)
              break;
            separated = separates(n.normalized());
          }
        }
        if (!separated) {
          resolved = false;
          ambiguous_q = slice.empty ? -888 : slice.q;
          ambiguous_box = box;
        }
      }
    }
    if (collision)
      return false;
    if (resolved)
      return true;
    if (depth <= 0 || sample_only) {
      if (!sample_only && ambiguous_box) {
        for (const auto& v : *ambiguous_box)
          for (const auto& w : path.crossSections(v))
            if (w.u >= u0 - 1e-10 && w.u <= u1 + 1e-10 && corridor.value(w.u, w.w) <= 1e-7) {
              if (result.witnesses.size() < max_witnesses)
                result.witnesses.push_back(w);
              collision = true;
            }
        if (collision)
          return false;
      }
      result.unresolved = true;
      if (result.unresolved_u < 0) {
        result.unresolved_u = mid;
        result.unresolved_error = error;
        result.unresolved_q = ambiguous_q;
      }
      return false;
    }
    V3 p01 = (control[0] + control[1]) * .5, p12 = (control[1] + control[2]) * .5,
       p23 = (control[2] + control[3]) * .5;
    V3 p012 = (p01 + p12) * .5, p123 = (p12 + p23) * .5, p = (p012 + p123) * .5;
    bool left = visit({control[0], p01, p012, p}, u0, mid, depth - 1);
    if (result.witnesses.size() >= max_witnesses)
      return false;
    bool right = visit({p, p123, p23, control[3]}, mid, u1, depth - 1);
    return left && right;
  };
  for (size_t i = 0; i < path.pieces(); ++i) {
    if (sample_only) {
      int count = std::max(2, int(std::ceil((path.controls()[i][3] - path.controls()[i][0]).norm() /
                                            (resolution_ * .25))));
      for (int j = 0; j <= count; ++j) {
        double u = (i + double(j) / count) / path.pieces();
        visit(path.controls()[i], u, u, 0);
        if (result.witnesses.size() >= max_witnesses)
          break;
      }
    } else if (!visit(path.controls()[i], double(i) / path.pieces(), double(i + 1) / path.pieces(),
                      max_depth))
      result.verified = false;
    if (result.witnesses.size() >= max_witnesses)
      break;
  }
  if (sample_only)
    result.verified = false;  // Sampling is only a cut oracle, never a certificate.
  return result;
}
}  // namespace corrgen
