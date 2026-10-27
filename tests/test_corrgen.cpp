#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "corrgen/corridor.hpp"
#include "corrgen/path.hpp"
#include "corrgen/solver.hpp"
#include "corrgen/voxel_planner.hpp"
using namespace corrgen;
void check(bool value, const char* message) {
  if (!value)
    throw std::runtime_error(message);
}
template <class F>
void rejects(F f) {
  bool caught = false;
  try {
    f();
  } catch (const std::exception&) {
    caught = true;
  }
  check(caught, "Expected failure");
}
int main() {
  try {
    Eigen::MatrixXd c = Eigen::MatrixXd::Zero(5, 4);
    c.col(0) << 2, 3, -0.3, 0.5, -0.7;
    c.col(1) << 0.2, -0.1, 0.1, 0.2, 0.1;
    c.col(2) << 0.05, 0.03, -0.01, 0.1, -0.1;
    Corridor corridor(c);
    check(corridor.certifyPositive(), "SPD certificate");
    for (double u : {0.0, 0.13, 0.5, 0.87, 1.0}) {
      auto b = chebyshev(3, u);
      double x = 2 * u - 1;
      check(std::abs(b.value[3] - (4 * x * x * x - 3 * x)) < 1e-12, "Chebyshev value");
      check(std::abs(b.first[3] - (24 * x * x - 6)) < 1e-12, "Chebyshev derivative");
      check(std::abs(b.second[3] - 96 * x) < 1e-12, "Chebyshev second derivative");
      auto e = corridor.ellipse(u);
      for (int j = 0; j < 50; ++j) {
        double a = 2 * std::acos(-1.) * j / 50;
        Eigen::Vector2d w =
            e.center +
            e.rotation *
                (e.radii.array() * Eigen::Vector2d(std::cos(a), std::sin(a)).array()).matrix();
        check(std::abs(corridor.evaluate(u, w).value) < 1e-12, "Offset ellipse boundary");
      }
    }
    Eigen::Vector3d x(0.37, 0.7, -0.4);
    const double h = 1e-5;
    auto d = corridor.evaluate(x[0], x.tail<2>());
    for (int k = 0; k < 3; ++k) {
      auto xp = x, xm = x;
      xp[k] += h;
      xm[k] -= h;
      auto dp = corridor.evaluate(xp[0], xp.tail<2>()), dm = corridor.evaluate(xm[0], xm.tail<2>());
      check(std::abs((dp.value - dm.value) / (2 * h) - d.gradient[k]) < 1e-7,
            "Gradient finite difference");
      check(((dp.gradient - dm.gradient) / (2 * h) - d.hessian.col(k)).norm() < 1e-7,
            "Hessian finite difference");
    }
    // Negative offdiagonal must not bypass strict DD.
    Eigen::MatrixXd bad = Eigen::MatrixXd::Zero(5, 1);
    bad.col(0) << 1, 1, -2, 0, 0;
    check(!Corridor(bad).certifyPositive(), "Negative-cross-term rejection");
    // Positive endpoints, negative interior: a=(u-.5)^2-.01.
    Eigen::MatrixXd dip = Eigen::MatrixXd::Zero(5, 3);
    dip(0, 0) = .115;
    dip(0, 2) = .125;
    dip(1, 0) = 1;
    check(!Corridor(dip).certifyPositive(), "Interior nonpositive interval");
    CubicPath path({Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 0), Eigen::Vector3d(2, 1, 0.3),
                    Eigen::Vector3d(3, 0, 0)});
    for (int i = 0; i < 10; ++i) {
      double u = .05 + .09 * i;
      auto f = path.frame(u);
      auto p = f.position + .05 * f.rotation.col(1);
      auto occ = path.project(p);
      check(std::abs(occ.u - u) < 1e-8, "Cubic exact projection");
      check((occ.w - Eigen::Vector2d(.05, 0)).norm() < 1e-8, "Projection transverse");
    }
    check(path.project(Eigen::Vector3d(-2, 0, 0)).u == 0, "Endpoint projection");
    rejects([&] { corridor.section(-.1); });
    rejects([&] { generate({{.5, Eigen::Vector2d::Zero()}}); });
    CubicPath straight({Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0),
                        Eigen::Vector3d(2, 0, 0), Eigen::Vector3d(3, 0, 0)});
    check(straight.crossSections(Eigen::Vector3d(-2, 0, 0)).empty(), "Beyond finite end cap");
    auto cross = straight.crossSections(Eigen::Vector3d(1.5, 2, 1));
    check(cross.size() == 1 && std::abs(cross[0].u - .5) < 1e-12, "Straight transverse section");
    std::vector<Eigen::Vector3d> wall;
    for (int j = 0; j < 8; ++j) wall.emplace_back(1.1, .1 + .2 * j, 1);
    CubicPath bending({Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 5, 0),
                       Eigen::Vector3d(2, -5, 0), Eigen::Vector3d(3, 0, 0)});
    auto multiple = bending.crossSections(Eigen::Vector3d(1.5, 0, 1));
    check(multiple.size() >= 3, "Enumerate multiple transverse intersections");
    SplinePath reverse({{3, 0, 1}, {2, 0, 1}, {1, 1, 1}, {0, 1, 1}, {0, 2, 2}});
    check((reverse.position(0) - Eigen::Vector3d(3, 0, 1)).norm() < 1e-12, "Full reference start");
    check((reverse.position(1) - Eigen::Vector3d(0, 2, 2)).norm() < 1e-12, "Full reference goal");
    for (size_t k = 1; k < reverse.pieces(); ++k) {
      double u = double(k) / reverse.pieces();
      check((reverse.position(u - 1e-8) - reverse.position(u + 1e-8)).norm() < 1e-5,
            "Reference C0");
      check((reverse.derivative(u - 1e-8) - reverse.derivative(u + 1e-8)).norm() < 1e-5,
            "Reference C1");
      check((reverse.derivative(u - 1e-8, 2) - reverse.derivative(u + 1e-8, 2)).norm() < 1e-4,
            "Reference C2");
      check((reverse.frame(u - 1e-8).rotation - reverse.frame(u + 1e-8).rotation).norm() < 1e-5,
            "Continuous frame");
    }
    SplinePath vertical({{1, 1, .2}, {1, 1, 1}, {1, 1, 2}});
    check(vertical.frame(.5).rotation.allFinite(), "Vertical reference frame");
    VoxelPlanner voxel(wall, {0, 0, 0}, {3, 3, 3}, .2, 0);
    auto route3d = voxel.search({2.7, .3, .5}, {.3, .3, 2.5});
    auto smooth = voxel.reference(route3d);
    check((smooth.position(0) - route3d.front()).norm() < 1e-12, "Reverse 3D start retained");
    check((smooth.position(1) - route3d.back()).norm() < 1e-12, "Reverse 3D goal retained");
    for (const auto& piece : smooth.controls())
      check(voxel.curveFree(piece), "Reference hull free of occupied voxels");
    std::vector<Eigen::Vector3d> barrier;
    for (int y = 0; y < 10; ++y)
      for (int z = 0; z < 10; ++z) barrier.emplace_back(1.1, .1 + .2 * y, .1 + .2 * z);
    VoxelPlanner blocked(barrier, {0, 0, 0}, {2, 2, 2}, .2, 0);
    rejects([&] { blocked.search({.3, .3, .3}, {1.7, .3, .3}); });
    check(!blocked.curveFree({Eigen::Vector3d(.3, .3, .3), Eigen::Vector3d(.7, .3, .3),
                              Eigen::Vector3d(1.3, .3, .3), Eigen::Vector3d(1.7, .3, .3)}),
          "Curve crossing voxel wall rejected");
    Options options;
    options.degree = 3;
    options.samples = 81;
    std::vector<Occupancy> cloud;
    for (int i = 0; i < 100; ++i) {
      double u = i / 99.;
      double angle = i * 2.399963;
      cloud.push_back({u, Eigen::Vector2d(1.0 * std::cos(angle), 1.5 * std::sin(angle))});
    }
    auto result = generate(cloud, options);
    check(result.min_obstacle_residual > 0, "Solved obstacle exclusion");
    for (int i = 0; i <= 1000; ++i) {
      auto s = result.corridor.section(i / 1000.);
      check(s.E.determinant() > 0 && s.E(0, 0) > 0, "Dense SPD");
    }
    for (int i = 0; i < 100; ++i) {
      double u = (i + .5) / 100.;
      Eigen::Vector2d w(.01 * i, .3 - .005 * i);
      check(std::abs(corridor.value(u, w) - corridor.evaluate(u, w).value) < 1e-9,
            "Value-only evaluator agrees");
    }
    for (int interval = 0; interval < 10; ++interval) {
      double lo = interval / 10., hi = (interval + 1) / 10.;
      double bound = corridor.radiusBound(lo, hi);
      auto enclosure = corridor.enclosingEllipse(lo, hi);
      for (int j = 0; j <= 10; ++j) {
        auto ellipse = corridor.ellipse(lo + (hi - lo) * j / 10.);
        for (int k = 0; k < 36; ++k) {
          double angle = k * 2 * std::acos(-1.) / 36;
          Eigen::Vector2d w =
              ellipse.center +
              ellipse.rotation * (ellipse.radii.array() *
                                  Eigen::Vector2d(std::cos(angle), std::sin(angle)).array())
                                     .matrix();
          check(w.norm() <= bound, "Interval enclosure contains ellipses");
          Eigen::Vector2d local = enclosure.rotation.transpose() * (w - enclosure.center);
          check((local.array() / enclosure.radii.array()).square().sum() <= 1 + 1e-10,
                "Directional interval ellipse contains every sampled original section");
        }
      }
    }
    Eigen::MatrixXd round = Eigen::MatrixXd::Zero(5, 1);
    round(0, 0) = round(1, 0) = 100;
    Corridor narrow(round);
    check(std::abs(narrow.radiusBound(0, 1) - .1) < 1e-7, "Constant radius bound");
    SplinePath tube_path({{.5, .5, 1.1}, {1.5, .5, 1.1}, {2.5, .5, 1.1}, {3.5, .5, 1.1}});
    VoxelPlanner solids({{2.1, 1.1, 1.1}}, {0, 0, 0}, {4, 3, 3}, .2, 0);
    check(solids.corridorFree(tube_path, narrow), "Continuous narrow tube free");
    round(0, 0) = round(1, 0) = 1.;
    check(!solids.corridorFree(tube_path, Corridor(round)),
          "Swept tube obstacle or boundary rejected");
    VoxelPlanner eroded({}, {0, 0, 0}, {4, 3, 3}, .2, .2);
    check(!eroded.free({.1, 1, 1}), "Robot boundary erosion");
    auto eroded_route = eroded.search({.5, .5, 1.1}, {3.5, 2.5, 1.1});
    for (const auto& v : eroded_route) check(eroded.free(v), "Astar respects eroded boundary");
    auto reserved = solids.reference({{.5, .5, 1.1}, {3.5, .5, 1.1}}, .08);
    for (const auto& control : reserved.controls())
      check(solids.curveFree(control, 12, .08), "Reference reserves continuous clearance");
    SplinePath flat_path({{.1, 1.5, 1.5}, {1.5, 1.5, 1.5}, {2.5, 1.5, 1.5}, {3.9, 1.5, 1.5}});
    Eigen::MatrixXd circle = Eigen::MatrixXd::Zero(5, 1);
    circle(0, 0) = circle(1, 0) = 1 / .64;
    Corridor wide(circle);
    VoxelPlanner corner_box({{2.1, 2.3, 2.3}}, {0, 0, 0}, {4, 4, 3}, .2, 0);
    check(!corner_box.corridorFree(flat_path, wide),
          "Old spherical enclosure rejects endpoint tangent extent");
    auto directional = corner_box.checkCorridor(flat_path, wide);
    check(directional.verified && directional.witnesses.empty(),
          "Directional ellipse enclosure preserves wide free disk");
    check(!corner_box.checkCorridor(flat_path, wide, 0, 128, true).verified,
          "Sampling never returns a continuous certificate");
    rejects([&] { corner_box.checkCorridor(flat_path, wide, 12, 0); });
    VoxelPlanner hit_box({{2.1, 1.7, 1.7}}, {0, 0, 0}, {4, 4, 3}, .2, 0);
    auto hit = hit_box.checkCorridor(flat_path, wide);
    check(!hit.verified && !hit.witnesses.empty(), "Body collision generates LP witnesses");
    for (const auto& w : hit.witnesses)
      check(wide.value(w.u, w.w) <= 1e-7, "Collision witness is inside original ellipse");
    if (solverAvailable("highs")) {
      Options fast;
      fast.solver = "highs";
      fast.degree = 3;
      fast.samples = 41;
      auto working = generate(cloud, fast);
      fast.working_set = false;
      auto full = generate(cloud, fast);
      check(working.quality == "OPTIMAL_SOLVER_TOLERANCE" &&
                full.quality == "OPTIMAL_SOLVER_TOLERANCE",
            "HiGHS complete solve");
      check(std::abs(working.objective - full.objective) < 1e-4,
            "Working/full LP objectives agree");
      fast.budget_ms = .00001;
      bool callback = false;
      auto seed = generate(cloud, fast, [&](const auto& r) {
        callback = true;
        check(r.min_obstacle_residual >= fast.exclusion_margin - fast.feasibility_tolerance,
              "Callback receives verified incumbent");
      });
      check(callback && seed.quality == "FEASIBLE_BUDGET", "Budget keeps verified seed");
      if (solverAvailable("proxqp")) {
        fast.degree = 0;
        fast.samples = 21;
        fast.budget_ms = 1000;
        fast.solver = "highs";
        auto baseline = generate(cloud, fast);
        fast.solver = "proxqp";
        auto trial = generate(cloud, fast);
        check(std::abs(trial.objective - baseline.objective) < 1e-3, "ProxQP versus LP baseline");
      }
    }
#ifdef CORRGEN_TEST_SDLP
    Options constant;
    constant.degree = 0;
    constant.samples = 21;
    const auto baseline = generate(cloud, constant);
    constant.solver = "sdlp";
    const auto trial = generate(cloud, constant);
    check(std::abs(baseline.objective - trial.objective) < 1e-5, "SDLP/GLPK objective agreement");
    check(trial.min_obstacle_residual > 0, "SDLP original obstacle residual");
    rejects([&] { generate({{.5, Eigen::Vector2d::Zero()}}, constant); });
    constant.degree = 2;
    rejects([&] { generate(cloud, constant); });
#endif
    std::cout << "PASS: basis, offset geometry, derivatives, Hessian, interval certificate, cubic "
                 "projection, invalid/infeasible inputs, LP exclusion\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
