#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  //get position X,Y
  vector<double> Get_X() {
      return position_x_;
  }

  vector<double> Get_Y() {
      return position_y_;
  }

 private:
  vector<double> position_x_, position_y_;
};

#endif /* MPC_H */
