// Optional experiment: compile the actual real_planner implementation in this TU
// so dimensions beyond its existing explicit <1>/<4> instantiations are available.
#include CORRGEN_SDLP_SOURCE
#include <glpk.h>

#include <chrono>
#include <iostream>
#include <vector>

#include "corrgen/corridor.hpp"

template <int D>
int compare() {
  constexpr int n = D / 5;
  std::vector<Eigen::Matrix<double, 1, D>> rows;
  std::vector<double> bounds;
  Eigen::Matrix<double, D, 1> cost = Eigen::Matrix<double, D, 1>::Zero();
  auto add = [&](double u, std::array<double, 5> w, double b) {
    auto t = corrgen::chebyshev(n - 1, u).value;
    Eigen::Matrix<double, 1, D> row;
    for (int r = 0; r < 5; ++r)
      for (int k = 0; k < n; ++k) row[r * n + k] = -w[r] * t[k];
    rows.push_back(row);
    bounds.push_back(-b);
  };
  for (int i = 0; i < 41; ++i) {
    double u = i / 40.;
    auto t = corrgen::chebyshev(n - 1, u).value;
    cost.template segment<n>(0) += t;
    cost.template segment<n>(n) += t;
    for (int sign : {-1, 1}) {
      add(u, {1, 0, double(sign), 0, 0}, .02);
      add(u, {0, 1, double(sign), 0, 0}, .02);
    }
    for (int j = 0; j < 16; ++j) {
      double a = 2 * std::acos(-1.) * j / 16;
      double y = (1 + .2 * u) * std::cos(a), z = (1.3 - .1 * u) * std::sin(a);
      add(u, {y * y, z * z, 2 * y * z, y, z}, 1.00001);
    }
  }
  Eigen::Matrix<double, Eigen::Dynamic, D> A(rows.size(), D);
  Eigen::VectorXd b(bounds.size());
  for (size_t i = 0; i < rows.size(); ++i) {
    A.row(i) = rows[i];
    b[i] = bounds[i];
  }
  for (int i = 0; i < A.size(); ++i)
    if (std::abs(A.data()[i]) < 1e-12)
      A.data()[i] = 0;
  glp_prob* lp = glp_create_prob();
  glp_add_cols(lp, D);
  glp_add_rows(lp, A.rows());
  std::vector<int> index(D + 1);
  std::vector<double> value(D + 1);
  for (int j = 0; j < D; ++j) {
    index[j + 1] = j + 1;
    glp_set_col_bnds(lp, j + 1, GLP_FR, 0, 0);
    glp_set_obj_coef(lp, j + 1, cost[j]);
  }
  for (int i = 0; i < A.rows(); ++i) {
    glp_set_row_bnds(lp, i + 1, GLP_UP, 0, b[i]);
    for (int j = 0; j < D; ++j) value[j + 1] = A(i, j);
    glp_set_mat_row(lp, i + 1, D, index.data(), value.data());
  }
  glp_smcp settings;
  glp_init_smcp(&settings);
  settings.msg_lev = GLP_MSG_OFF;
  settings.presolve = GLP_ON;
  auto begin = std::chrono::steady_clock::now();
  int code = glp_simplex(lp, &settings);
  double objective = glp_get_obj_val(lp);
  std::cout
      << "dimension=" << D << " rows=" << A.rows() << " glpk_code=" << code
      << " glpk_objective=" << objective << " glpk_ms="
      << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count()
      << std::endl;
  if (code || glp_get_status(lp) != GLP_OPT) {
    glp_delete_prob(lp);
    return 1;
  }
  glp_delete_prob(lp);
  Eigen::Matrix<double, D, 1> x;
  begin = std::chrono::steady_clock::now();
  std::cout << "starting real_planner SDLP..." << std::endl;
  double sdlp = math_utils::sdlp::linprog<D>(cost, A, b, x);
  double residual = (A * x - b).maxCoeff();
  std::cout
      << "sdlp_objective=" << sdlp << " sdlp_ms="
      << std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count()
      << " max_violation=" << residual << std::endl;
  return std::isfinite(sdlp) && residual < 1e-6 && std::abs(sdlp - objective) < 1e-4 ? 0 : 1;
}
int main(int argc, char** argv) {
  int degree = argc > 1 ? std::stoi(argv[1]) : 0;
  if (degree == 0)
    return compare<5>();
  if (degree == 3)
    return compare<20>();
  if (degree == 6)
    return compare<35>();
  return 2;
}
