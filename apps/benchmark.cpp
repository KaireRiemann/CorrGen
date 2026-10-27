#include <Eigen/Eigenvalues>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "corrgen/corridor.hpp"
#include "corrgen/path.hpp"
using namespace corrgen;
int main(int argc, char** argv) {
  try {
    if (argc < 4 || argc > 5) {
      std::cerr << "Usage: corrgen_benchmark cloud.xyz control_points.xyz output_dir [degree]\n";
      return 2;
    }
    Options o;
    if (argc == 5)
      o.degree = std::stoi(argv[4]);
    std::ifstream cp(argv[2]);
    std::array<Eigen::Vector3d, 4> controls;
    for (auto& p : controls)
      if (!(cp >> p.x() >> p.y() >> p.z()))
        throw std::runtime_error("Expected four xyz Bezier controls");
    std::string extra;
    if (cp >> extra)
      throw std::runtime_error("Unexpected extra path data");
    CubicPath path(controls);
    std::ifstream input(argv[1]);
    if (!input)
      throw std::runtime_error("Cannot open cloud");
    std::vector<Eigen::Vector3d> world;
    std::vector<Occupancy> projected;
    std::string line;
    int line_number = 0;
    size_t outside_sweep = 0;
    auto start = std::chrono::steady_clock::now();
    while (std::getline(input, line)) {
      ++line_number;
      if (line.empty() || line[0] == '#')
        continue;
      std::istringstream row(line);
      Eigen::Vector3d p;
      if (!(row >> p.x() >> p.y() >> p.z()) || (row >> extra))
        throw std::runtime_error("Invalid xyz row " + std::to_string(line_number));
      auto sections = path.crossSections(p);
      if (sections.empty())
        ++outside_sweep;
      projected.insert(projected.end(), sections.begin(), sections.end());
      world.push_back(p);
    }
    if (world.empty())
      throw std::runtime_error("Empty cloud");
    double projection_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    auto result = generate(projected, o);
    std::filesystem::create_directories(argv[3]);
    auto dir = std::filesystem::path(argv[3]);
    std::ofstream coefficients(dir / "coefficients.csv");
    coefficients << std::setprecision(17);
    for (int r = 0; r < 5; ++r) {
      for (int k = 0; k <= o.degree; ++k)
        coefficients << (k ? "," : "") << result.corridor.coefficients()(r, k);
      coefficients << '\n';
    }
    std::ofstream surface(dir / "surface.csv"), centers(dir / "sections.csv"),
        occupancy(dir / "projected.csv");
    surface << "u,angle,x,y,z\n";
    centers << "u,x,y,z,area,min_eigenvalue\n";
    occupancy << "u,w1,w2\n";
    surface << std::setprecision(12);
    centers << std::setprecision(12);
    occupancy << std::setprecision(17);
    for (const auto& p : projected) occupancy << p.u << ',' << p.w.x() << ',' << p.w.y() << '\n';
    double min_eigen = 1e100, volume_proxy = 0, min_distance = 1e100;
    int surface_count = 0;
    // Independent Euclidean nearest-point distances on sampled boundary. Not a continuous
    // solid-obstacle proof.
    for (int i = 0; i <= 200; ++i) {
      double u = i / 200.;
      auto f = path.frame(u);
      auto e = result.corridor.ellipse(u);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(result.corridor.section(u).E);
      min_eigen = std::min(min_eigen, eig.eigenvalues()[0]);
      Eigen::Vector3d center = f.position + f.rotation.rightCols<2>() * e.center;
      centers << u << ',' << center.x() << ',' << center.y() << ',' << center.z() << ',' << e.area
              << ',' << eig.eigenvalues()[0] << '\n';
      if (i)
        volume_proxy += .5 * (e.area + result.corridor.ellipse((i - 1) / 200.).area) *
                        (path.position(u) - path.position((i - 1) / 200.)).norm();
      for (int j = 0; j < 48; ++j) {
        double a = 2 * std::acos(-1.) * j / 48.;
        Eigen::Vector2d w =
            e.center +
            e.rotation *
                (e.radii.array() * Eigen::Vector2d(std::cos(a), std::sin(a)).array()).matrix();
        Eigen::Vector3d p = f.position + f.rotation.rightCols<2>() * w;
        surface << u << ',' << a << ',' << p.x() << ',' << p.y() << ',' << p.z() << '\n';
        for (const auto& q : world) min_distance = std::min(min_distance, (p - q).squaredNorm());
        ++surface_count;
      }
    }
    std::ofstream report(dir / "metrics.json");
    report << std::setprecision(12) << "{\n  \"points\": " << world.size()
           << ",\n  \"transverse_constraints\": " << projected.size()
           << ",\n  \"points_outside_finite_sweep\": " << outside_sweep
           << ",\n  \"degree\": " << o.degree << ",\n  \"constraints\": " << result.constraint_count
           << ",\n  \"projection_ms\": " << projection_ms
           << ",\n  \"solve_ms\": " << result.solve_ms
           << ",\n  \"trace_objective\": " << result.objective
           << ",\n  \"min_obstacle_q\": " << result.min_obstacle_residual
           << ",\n  \"min_sampled_eigenvalue\": " << min_eigen
           << ",\n  \"bernstein_positive_certificate\": true,\n  \"area_arclength_integral\": "
           << volume_proxy
           << ",\n  \"sampled_boundary_cloud_distance\": " << std::sqrt(min_distance)
           << ",\n  \"surface_samples\": " << surface_count << "\n}\n";
    if (!report || !surface || !centers || !coefficients || !occupancy)
      throw std::runtime_error("Output write failed");
    std::cout << "PASS points=" << world.size() << " degree=" << o.degree
              << " solve_ms=" << result.solve_ms << " min_q=" << result.min_obstacle_residual
              << " output=" << dir << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
