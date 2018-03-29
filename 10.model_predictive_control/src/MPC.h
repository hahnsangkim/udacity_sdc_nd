#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
 	//size_t N_;
	//double dt_;
  vector<double> mpc_x;
  vector<double> mpc_y;

  MPC();

  virtual ~MPC();

  // x and y for predicted path
  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
