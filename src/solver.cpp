#include "corrgen/solver.hpp"

#include <glpk.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "sdlp_adapter.hpp"
#ifdef CORRGEN_HIGHS
#include <Highs.h>
#endif
namespace corrgen {
#ifdef CORRGEN_PROXQP
LinearSolution solveProxqp(const LinearProblem&, const Eigen::VectorXd&, double);
#endif
bool solverAvailable(const std::string& name) {
  if (name == "glpk")
    return true;
#ifdef CORRGEN_SDLP_SOURCE
  if (name == "sdlp")
    return true;
#endif
#ifdef CORRGEN_HIGHS
  if (name == "highs")
    return true;
#endif
#ifdef CORRGEN_PROXQP
  if (name == "proxqp")
    return true;
#endif
  return false;
}
LinearSolution solveLinear(const LinearProblem& p, const std::string& name,
                           const Eigen::VectorXd& initial, double budget_ms) {
  if (!solverAvailable(name))
    throw std::invalid_argument("Backend not available: " + name);
  if (p.A.cols() != p.cost.size() || p.A.rows() != p.lower.size() || !p.A.allFinite() ||
      !p.lower.allFinite() || !p.cost.allFinite() || budget_ms <= 0)
    throw std::invalid_argument("Invalid linear problem");
  const auto begin = std::chrono::steady_clock::now();
  LinearSolution out;
#ifdef CORRGEN_PROXQP
  if (name == "proxqp")
    return solveProxqp(p, initial, budget_ms);
#endif
  if (name == "sdlp") {
    out.x = solveSdlp(-p.A, -p.lower, p.cost);
    out.status = "SDLP_FINITE";  // Caller checks original residuals; no dual certificate.
  } else if (name == "glpk") {
    std::unique_ptr<glp_prob, decltype(&glp_delete_prob)> lp(glp_create_prob(), glp_delete_prob);
    glp_set_obj_dir(lp.get(), GLP_MIN);
    glp_add_cols(lp.get(), p.A.cols());
    glp_add_rows(lp.get(), p.A.rows());
    std::vector<int> indices(p.A.cols() + 1);
    std::vector<double> values(p.A.cols() + 1);
    for (int j = 0; j < p.A.cols(); ++j) {
      indices[j + 1] = j + 1;
      glp_set_col_bnds(lp.get(), j + 1, GLP_FR, 0, 0);
      glp_set_obj_coef(lp.get(), j + 1, p.cost[j]);
    }
    for (int i = 0; i < p.A.rows(); ++i) {
      glp_set_row_bnds(lp.get(), i + 1, GLP_LO, p.lower[i], 0);
      for (int j = 0; j < p.A.cols(); ++j) values[j + 1] = p.A(i, j);
      glp_set_mat_row(lp.get(), i + 1, p.A.cols(), indices.data(), values.data());
    }
    glp_smcp options;
    glp_init_smcp(&options);
    options.msg_lev = GLP_MSG_OFF;
    options.presolve = GLP_ON;
    options.tol_bnd = 1e-8;
    options.tol_dj = 1e-8;
    options.tm_lim = std::max(1, int(std::min(2147483000.0, budget_ms)));
    int code = glp_simplex(lp.get(), &options), status = glp_get_status(lp.get());
    out.status = "GLPK_" + std::to_string(code) + "_" + std::to_string(status);
    out.optimal = code == 0 && status == GLP_OPT;
    if (status == GLP_FEAS || status == GLP_OPT) {
      out.x.resize(p.A.cols());
      for (int j = 0; j < p.A.cols(); ++j) out.x[j] = glp_get_col_prim(lp.get(), j + 1);
    }
    out.iterations = glp_get_it_cnt(lp.get());
#ifdef CORRGEN_HIGHS
  } else if (name == "highs") {
    Highs highs;
    highs.setOptionValue("output_flag", false);
    highs.setOptionValue("threads", 1);
    highs.setOptionValue("solver", "simplex");
    highs.setOptionValue("presolve", "off");
    highs.setOptionValue("time_limit", budget_ms / 1000.);
    highs.setOptionValue("primal_feasibility_tolerance", 1e-9);
    highs.setOptionValue("dual_feasibility_tolerance", 1e-9);
    HighsLp lp;
    lp.num_col_ = p.A.cols();
    lp.num_row_ = p.A.rows();
    lp.col_cost_.assign(p.cost.data(), p.cost.data() + p.cost.size());
    lp.col_lower_.assign(lp.num_col_, -kHighsInf);
    lp.col_upper_.assign(lp.num_col_, kHighsInf);
    lp.row_lower_.assign(p.lower.data(), p.lower.data() + p.lower.size());
    lp.row_upper_.assign(lp.num_row_, kHighsInf);
    lp.a_matrix_.format_ = MatrixFormat::kRowwise;
    lp.a_matrix_.start_.assign(1, 0);
    for (int i = 0; i < p.A.rows(); ++i) {
      for (int j = 0; j < p.A.cols(); ++j)
        if (p.A(i, j) != 0) {
          lp.a_matrix_.index_.push_back(j);
          lp.a_matrix_.value_.push_back(p.A(i, j));
        }
      lp.a_matrix_.start_.push_back(lp.a_matrix_.value_.size());
    }
    if (highs.passModel(lp) == HighsStatus::kError)
      throw std::runtime_error("HiGHS rejected LP model");
    if (initial.size() == p.A.cols()) {
      HighsSolution warm;
      warm.col_value.assign(initial.data(), initial.data() + initial.size());
      warm.value_valid = true;
      highs.setSolution(warm);
    }
    highs.run();
    out.status = highs.modelStatusToString(highs.getModelStatus());
    out.optimal = highs.getModelStatus() == HighsModelStatus::kOptimal;
    const auto& solution = highs.getSolution();
    if (solution.value_valid)
      out.x =
          Eigen::Map<const Eigen::VectorXd>(solution.col_value.data(), solution.col_value.size());
    out.iterations = highs.getInfo().simplex_iteration_count;
#endif
  }
  out.milliseconds =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
  return out;
}
}  // namespace corrgen
