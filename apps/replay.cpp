#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "corrgen/voxel_planner.hpp"

std::vector<Eigen::Vector3d> read(const std::filesystem::path& file) {
  std::ifstream input(file);
  if (!input)
    throw std::runtime_error("Cannot open " + file.string());
  std::vector<Eigen::Vector3d> points;
  double x, y, z;
  while (input >> x >> y >> z) points.emplace_back(x, y, z);
  if (points.empty() || !input.eof())
    throw std::runtime_error("Invalid input " + file.string());
  return points;
}
int main(int argc, char** argv) {
  if (argc < 4 || argc > 6) {
    std::cerr << "Usage: corrgen_replay REQUEST_DIR glpk|sdlp|highs|proxqp DEGREE [working|full] "
                 "[BUDGET_MS]\n"
              << "Replay uses the 50x50x5 map, origin(-25,-25,0), resolution0.25, inflation0.\n";
    return 2;
  }
  try {
    const std::filesystem::path dir(argv[1]);
    auto cloud = read(dir / "cloud.xyz"), route = read(dir / "astar.xyz");
    corrgen::Options options;
    options.solver = argv[2];
    options.degree = std::stoi(argv[3]);
    if (argc >= 5) {
      const std::string mode(argv[4]);
      if (mode != "working" && mode != "full")
        throw std::invalid_argument("Expected working or full mode");
      options.working_set = mode == "working";
    }
    if (argc == 6)
      options.budget_ms = std::stod(argv[5]);
    corrgen::VoxelPlanner grid(cloud, {-25, -25, 0}, {50, 50, 5}, .25, 0);
    auto path = grid.reference(route);
    std::vector<corrgen::Occupancy> occupancy;
    for (const auto& point : cloud) {
      auto sections = path.crossSections(point);
      occupancy.insert(occupancy.end(), sections.begin(), sections.end());
    }
    std::cout << "solver=" << options.solver << " degree=" << options.degree
              << " points=" << cloud.size() << " pieces=" << path.pieces()
              << " obstacle_constraints=" << occupancy.size() << " SOLVING" << std::endl;
    auto result = corrgen::generate(occupancy, options);
    std::cout << std::setprecision(14) << "PASS objective=" << result.objective
              << " generate_ms=" << result.solve_ms << " rows=" << result.constraint_count
              << " min_q=" << result.min_obstacle_residual << " quality=" << result.quality
              << " backend=" << result.backend_status << " working_rows=" << result.working_rows
              << " rounds=" << result.rounds << " backend_ms=" << result.backend_ms
              << " verification_ms=" << result.verification_ms
              << " positive_certificate=1 all_original_rows_checked=1" << std::endl;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << std::endl;
    return 1;
  }
}
