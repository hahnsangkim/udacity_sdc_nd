#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;
using namespace std;

// TODO: Set the timestep length and duration
size_t N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
double rcte = 0;
double repsi = 0;
double rv = 55; //speed

// starting points for solver
size_t x_start      = 0 ;
size_t y_start      = x_start     + N ;
size_t psi_start    = y_start     + N ;
size_t v_start      = psi_start   + N ;
size_t cte_start    = v_start     + N ;
size_t epsi_start   = cte_start   + N ;
size_t delta_start  = epsi_start  + N ;
size_t a_start      = delta_start + N - 1 ;

// adjustment values for cost function
const double coeff_c_rcte           = .8;
const double coeff_c_repsi          = 200;
const double coeff_c_rv             = .8;
const double coeff_c_rval_throttle  = 10;
const double coeff_c_rval_steering  = 10;
const double coeff_c_rseq_throttle  = 500;
const double coeff_c_rseq_steering  = 10; //1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    /*** cost vectors ***/
    fg[0] = 0;

    // Cost for reference state
    for (int t=0; t < N; t++){
      fg[0] += coeff_c_rcte * CppAD::pow(vars[cte_start + t] - rcte, 2);
      fg[0] += coeff_c_repsi * CppAD::pow(vars[epsi_start + t] - repsi, 2);
      fg[0] += coeff_c_rv * CppAD::pow(vars[v_start + t] - rv, 2);
    }

    // Reduce actuators values
    for (int t=0; t < (N-1); t++){
      fg[0] += coeff_c_rval_steering * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += coeff_c_rval_throttle * CppAD::pow(vars[a_start + t], 2);
    }

    // Reduce gap between actuations (sequence)
    for (int t=0; t < (N-2); t++){
      fg[0] += coeff_c_rseq_steering * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += coeff_c_rseq_throttle * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /*** Initialize constraints ***/
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    /****************************************
     * Kinematic model
     ****************************************/

    for (int i = 0; i < N - 1; i++) {
      // State time t + 1
      AD<double> x_t1 = vars[x_start + i + 1];
      AD<double> y_t1 = vars[y_start + i + 1];
      AD<double> psi_t1 = vars[psi_start + i + 1];
      AD<double> v_t1 = vars[v_start + i + 1];
      AD<double> cte_t1 = vars[cte_start + i + 1];
      AD<double> e_psi_t1 = vars[epsi_start + i + 1];
      // State at time t
      AD<double> x_t0 = vars[x_start + i];
      AD<double> y_t0 = vars[y_start + i];
      AD<double> v_t0 = vars[v_start + i];
      AD<double> psi_t0 = vars[psi_start + i];
      AD<double> cte_t0 = vars[cte_start + i];
      AD<double> epsi_t0 = vars[epsi_start + i];

      AD<double> delta_t0 = vars[delta_start + i];
      AD<double> a_t0 = vars[a_start + i];

      // for 3rd degree polynomial
      AD<double> f_t0 = coeffs[0] + coeffs[1] * x_t0 + 
                    coeffs[2] * pow(x_t0, 2) + coeffs[3] * pow(x_t0, 3);
      
      // rate of cahnge of f0
      AD<double> f_t0_rate_of_change = coeffs[1] + 2 * coeffs[2] * x_t0 + 
                           3 * coeffs[3] * pow(x_t0,2);       
       
      AD<double> psides_t0 = CppAD::atan( f_t0_rate_of_change);

      fg[2 + x_start + i] = x_t1 - (x_t0 + v_t0 * CppAD::cos(psi_t0) * dt);
      fg[2 + y_start + i] = y_t1 - (y_t0 + v_t0 * CppAD::sin(psi_t0) * dt);

      fg[2 + psi_start + i] = psi_t1 - (psi_t0 + v_t0 / Lf * delta_t0 * dt);
      fg[2 + v_start + i] = v_t1 - (v_t0 + a_t0 * dt);

      fg[2 + cte_start + i] = cte_t1 - ((f_t0-y_t0) + (v_t0 * CppAD::sin(epsi_t0) * dt));
      fg[2 + epsi_start + i] = e_psi_t1 - ((psi_t0 - psides_t0) + v_t0 * delta_t0 / Lf * dt ) ;
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  //this->N_ = N;
  //this->dt_ = dt;

}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  size_t n_vars = state.size() * N + 2 * (N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = state.size() * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.;
  }
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // borrowed from swirlingsand
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.;
    vars_upperbound[i] = 1.;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);
  //fg_eval.operator(vars);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  this->mpc_x = {};
  this->mpc_y = {};

  for (int i = 0; i < state.size();i++){
    this->mpc_x.push_back(solution.x[x_start+1+i]);
    this->mpc_y.push_back(solution.x[y_start+1+i]);
  }

  //cout << "steering_angle" << solution.x[delta_start] << 
  //"throttle" << solution.x[a_start] << endl;

  vector<double> results;

  results.push_back(solution.x[x_start + 1]);
  results.push_back(solution.x[y_start + 1]);
  results.push_back(solution.x[psi_start + 1]);
  results.push_back(solution.x[v_start + 1]);
  results.push_back(solution.x[cte_start + 1]);
  results.push_back(solution.x[epsi_start + 1]);
  results.push_back(solution.x[delta_start]);
  results.push_back(solution.x[a_start]);

  return results;
}
