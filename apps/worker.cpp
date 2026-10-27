#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <thread>

#include "corrgen/voxel_planner.hpp"
using Json = nlohmann::json;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
double ms(Clock::time_point t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
void atomicJson(const fs::path& path, const Json& data) {
  auto tmp = path;
  tmp += ".tmp";
  {
    std::ofstream f(tmp);
    f << data.dump(2) << '\n';
    if (!f)
      throw std::runtime_error("Cannot write " + tmp.string());
  }
  fs::rename(tmp, path);
}
std::vector<Eigen::Vector3d> readCloud(const fs::path& file) {
  std::ifstream f(file);
  if (!f)
    throw std::runtime_error("Cannot read cloud");
  double x, y, z;
  std::vector<Eigen::Vector3d> out;
  while (f >> x >> y >> z) out.emplace_back(x, y, z);
  if (!f.eof() || out.empty())
    throw std::runtime_error("Invalid cloud file");
  return out;
}
Eigen::Vector3d vec(const Json& a) {
  return {a.at(0).get<double>(), a.at(1).get<double>(), a.at(2).get<double>()};
}
void writePoints(const fs::path& file, const std::vector<Eigen::Vector3d>& points) {
  std::ofstream f(file);
  f << std::setprecision(17);
  for (const auto& p : points) f << p.transpose() << '\n';
  if (!f)
    throw std::runtime_error("Cannot write points");
}
int main(int argc, char** argv) {
  if (argc != 2)
    return 2;
  fs::path dir = argv[1];
  Json metrics;
  auto started = Clock::now();
  auto phase = [&](const std::string& name) {
    atomicJson(dir / "progress.json", {{"phase", name}, {"elapsed_ms", ms(started)}});
  };
  try {
    Json config;
    {
      std::ifstream f(dir / "request.json");
      f >> config;
    }
    metrics = {{"request", config.at("request")},
               {"map_version", config.at("map_version")},
               {"map_hash", config.at("map_hash")},
               {"config_hash", config.at("config_hash")},
               {"solver", config.at("solver")}};
    phase("READING_SNAPSHOT");
    auto t = Clock::now();
    auto cloud = readCloud(config.at("cloud_file").get<std::string>());
    double inflation = config.at("inflation").get<double>() +
                       config.at("robot_radius").get<double>() +
                       config.at("safety_margin").get<double>();
    corrgen::VoxelPlanner grid(cloud, vec(config.at("origin")), vec(config.at("size")),
                               config.at("resolution"), inflation,
                               config.value("clearance_weight", 4.));
    const double reference_clearance = config.value("reference_clearance", .1);
    corrgen::VoxelPlanner search_grid(cloud, vec(config.at("origin")), vec(config.at("size")),
                                      config.at("resolution"), inflation + reference_clearance,
                                      config.value("clearance_weight", 4.));
    metrics["reference_clearance"] = reference_clearance;
    metrics["clearance_weight"] = config.value("clearance_weight", 4.);
    metrics["map_ms"] = ms(t);
    metrics["inflation_total"] = inflation;
    metrics["robot_radius"] = config.at("robot_radius");
    metrics["safety_margin"] = config.at("safety_margin");
    metrics["cloud_points"] = cloud.size();
    phase("PLANNING_ASTAR");
    t = Clock::now();
    auto astar = search_grid.search(vec(config.at("start")), vec(config.at("goal")));
    metrics["astar_ms"] = ms(t);
    metrics["astar_points"] = astar.size();
    writePoints(dir / "astar.xyz", astar);
    phase("BUILDING_REFERENCE");
    t = Clock::now();
    auto path = grid.reference(astar, reference_clearance);
    metrics["reference_ms"] = ms(t);
    metrics["reference_pieces"] = path.pieces();
    std::vector<Eigen::Vector3d> reference;
    // Display sampling is independent of the continuous validation resolution.
    int samples = std::min(401, std::max(201, int(path.pieces() * 5 + 1)));
    for (int i = 0; i < samples; ++i) reference.push_back(path.position(double(i) / (samples - 1)));
    writePoints(dir / "reference.xyz", reference);
    {
      std::ofstream f(dir / "reference_bezier.csv");
      f << std::setprecision(17);
      for (const auto& c : path.controls())
        for (const auto& p : c) f << p.x() << ',' << p.y() << ',' << p.z() << '\n';
    }
    {
      std::ofstream f(dir / "frame_axis.xyz");
      f << std::setprecision(17) << path.frameAxis().transpose() << '\n';
    }
    phase("PROJECTING_CLOUD");
    t = Clock::now();
    int threads = std::max(1, std::min(8, config.value("projection_threads", 4)));
    std::vector<std::vector<corrgen::Occupancy>> chunks(threads);
    std::vector<size_t> outside(threads, 0);
    std::vector<std::thread> workers;
    std::exception_ptr error;
    std::mutex error_mutex;
    for (int k = 0; k < threads; ++k)
      workers.emplace_back([&, k] {
        try {
          for (size_t i = cloud.size() * k / threads; i < cloud.size() * (k + 1) / threads; ++i) {
            auto sections = path.crossSections(cloud[i]);
            outside[k] += sections.empty();
            chunks[k].insert(chunks[k].end(), sections.begin(), sections.end());
          }
        } catch (...) {
          std::lock_guard<std::mutex> guard(error_mutex);
          error = std::current_exception();
        }
      });
    for (auto& worker : workers) worker.join();
    if (error)
      std::rethrow_exception(error);
    std::vector<corrgen::Occupancy> occupancy;
    size_t outside_count = 0;
    for (int k = 0; k < threads; ++k) {
      occupancy.insert(occupancy.end(), chunks[k].begin(), chunks[k].end());
      outside_count += outside[k];
    }
    metrics["projection_ms"] = ms(t);
    metrics["outside_finite_sweep"] = outside_count;
    metrics["transverse_constraints"] = occupancy.size();
    corrgen::Options options;
    options.solver = config.at("solver");
    options.degree = config.at("degree");
    options.working_set = config.value("working_set", true);
    options.budget_ms =
        std::max(1., std::min(config.value("solver_budget_ms", 1000.),
                              config.value("job_budget_ms", 5000.) - ms(started) - 100.));
    phase("OPTIMIZING_CORRIDOR");
    Json best;
    double best_objective = std::numeric_limits<double>::infinity(), validation_ms = 0;
    int sequence = 0;
    auto accept = [&](const corrgen::SolveResult& result) {
      auto verification_start = Clock::now();
      corrgen::Corridor candidate = result.corridor;
      // Called only after directional continuous body validation succeeds.
      double min_q = std::numeric_limits<double>::infinity();
      for (const auto& p : occupancy) min_q = std::min(min_q, candidate.value(p.u, p.w));
      if (min_q < options.exclusion_margin - options.feasibility_tolerance ||
          !candidate.certifyPositive()) {
        validation_ms += ms(verification_start);
        return;
      }
      double objective = result.objective;
      validation_ms += ms(verification_start);
      if (objective >= best_objective)
        return;
      best_objective = objective;
      best = metrics;
      best["status"] = "VERIFIED";
      best["validation"] = "INFLATED_VOXEL_ENCLOSURE";
      best["validation_method"] = "DIRECTIONAL_INTERVAL_SUPPORT";
      best["geometry_cut_guard_m"] = .01;
      best["certificate_arithmetic"] = "long_double_with_roundoff_guard";
      best["sequence"] = ++sequence;
      best["objective"] = objective;
      best["min_q"] = min_q;
      best["shrink_factor"] = 1.;
      best["quality"] = "FEASIBLE_BODY_CONSTRAINED";
      best["solver_quality"] = result.quality;
      best["start_error"] = (path.position(0) - vec(config.at("start"))).norm();
      best["goal_error"] = (path.position(1) - vec(config.at("goal"))).norm();
      best["positive_certificate"] = true;
      best["reference_voxel_free"] = true;
      best["boundary_checked"] = true;
      best["validation_ms"] = validation_ms;
      best["total_ms"] = ms(started);
      best["coefficients"] = Json::array();
      for (int r = 0; r < 5; ++r) {
        std::vector<double> row;
        for (int j = 0; j < candidate.coefficients().cols(); ++j)
          row.push_back(candidate.coefficients()(r, j));
        best["coefficients"].push_back(row);
      }
      // Rings are only display geometry; the certificate covers the continuous enclosure.
      best["rings"] = Json::array();
      double min_radius = std::numeric_limits<double>::infinity(), area_sum = 0;
      std::vector<double> minor_axes, major_axes;
      for (int i = 0; i < samples; ++i) {
        double u = double(i) / (samples - 1);
        auto f = path.frame(u);
        auto ellipse = candidate.ellipse(u);
        min_radius = std::min(min_radius, ellipse.radii.minCoeff());
        minor_axes.push_back(ellipse.radii.minCoeff());
        major_axes.push_back(ellipse.radii.maxCoeff());
        area_sum += ellipse.area;
        Json ring = Json::array();
        for (int j = 0; j <= 32; ++j) {
          double a = 2 * std::acos(-1.) * j / 32;
          Eigen::Vector2d w = ellipse.center +
                              ellipse.rotation * (ellipse.radii.array() *
                                                  Eigen::Vector2d(std::cos(a), std::sin(a)).array())
                                                     .matrix();
          Eigen::Vector3d p = f.position + f.rotation.rightCols<2>() * w;
          ring.push_back({p.x(), p.y(), p.z()});
        }
        best["rings"].push_back(ring);
      }
      best["sampled_min_semiaxis_m"] = min_radius;
      std::sort(minor_axes.begin(), minor_axes.end());
      std::sort(major_axes.begin(), major_axes.end());
      best["sampled_median_minor_m"] = minor_axes[minor_axes.size() / 2];
      best["sampled_median_major_m"] = major_axes[major_axes.size() / 2];
      best["sampled_mean_area_m2"] = area_sum / samples;
      atomicJson(dir / "candidate.json", best);
    };
    std::optional<corrgen::SolveResult> accepted;
    const auto geometry_started = Clock::now();
    const size_t original_constraints = occupancy.size();
    size_t geometry_boxes = 0;
    Json trace = Json::array();
    double optimization_ms = 0;
    for (int iteration = 0; iteration < 48; ++iteration) {
      double remaining = config.value("job_budget_ms", 5000.) - ms(started) - 100.;
      if (remaining <= 0)
        break;
      options.budget_ms = std::min(config.value("solver_budget_ms", 1000.), remaining);
      auto trial = corrgen::generate(occupancy, options);
      optimization_ms += trial.solve_ms;
      const auto checked_at = Clock::now();
      auto check = grid.checkCorridor(path, trial.corridor, 0, 2048, true);
      if (check.witnesses.empty())
        check = grid.checkCorridor(path, trial.corridor, 16, 512);
      validation_ms += ms(checked_at);
      geometry_boxes += check.boxes_tested;
      trace.push_back({{"round", iteration + 1},
                       {"witnesses", check.witnesses.size()},
                       {"unresolved_u", check.unresolved_u},
                       {"unresolved_error", check.unresolved_error},
                       {"unresolved_q", check.unresolved_q},
                       {"boxes", check.boxes_tested},
                       {"elapsed_ms", ms(geometry_started)}});
      atomicJson(dir / "geometry_trace.json", trace);
      metrics["geometry_rounds"] = iteration + 1;
      metrics["geometry_constraints"] = occupancy.size() - original_constraints;
      metrics["geometry_boxes_checked"] = geometry_boxes;
      metrics["geometry_validation_ms"] = validation_ms;
      metrics["geometry_unresolved"] = check.unresolved;
      if (check.verified) {
        accept(trial);
        accepted = std::move(trial);
        break;
      }
      if (check.witnesses.empty())
        break;
      // A 1 cm inward-normal guard keeps cuts away from numerical tangency. These
      // extra constraints are an explicit geometry margin, not deleted obstacles.
      for (auto witness : check.witnesses) {
        double distance = witness.w.norm();
        if (distance < 1e-6)
          throw std::runtime_error("Reference intersects body obstacle");
        auto section = trial.corridor.section(witness.u);
        Eigen::Vector2d normal = 2 * section.E * witness.w + section.d;
        if (normal.norm() < 1e-12)
          normal = witness.w;
        witness.w -= std::min(.01, .25 * distance) * normal.normalized();
        occupancy.push_back(witness);
      }
      phase("REFINING_BODY_CONSTRAINTS");
    }
    metrics["optimization_total_ms"] = optimization_ms;
    metrics["geometry_pipeline_ms"] = ms(geometry_started);
    if (!accepted)
      throw std::runtime_error(
          "UNVERIFIED: body constraint refinement did not converge within budget; no global shrink "
          "fallback");
    auto& result = *accepted;
    metrics["solve_ms"] = result.solve_ms;
    metrics["backend_ms"] = result.backend_ms;
    metrics["full_check_ms"] = result.verification_ms;
    metrics["working_rows"] = result.working_rows;
    metrics["rounds"] = result.rounds;
    metrics["backend_status"] = result.backend_status;
    metrics["solver_quality"] = result.quality;
    metrics["validation_ms"] = validation_ms;
    if (best.empty())
      throw std::runtime_error("UNVERIFIED: no continuous voxel enclosure passed");
    best.update(metrics);
    best["status"] = "PASS";
    best["total_ms"] = ms(started);
    best["sequence"] = ++sequence;
    atomicJson(dir / "candidate.json", best);
    best.erase("rings");
    atomicJson(dir / "metrics.json", best);
    phase("COMPLETE");
    return 0;
  } catch (const std::exception& e) {
    metrics["status"] = "FAIL";
    metrics["error"] = e.what();
    metrics["total_ms"] = ms(started);
    atomicJson(dir / "failure.json", metrics);
    phase("FAILED");
    std::cerr << e.what() << '\n';
    return 1;
  }
}
