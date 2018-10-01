#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

namespace Tools {

  const float EPSILON	= 0.000001;

  /**
  * convert radar measurements from cartesian coordinates (x, y, vx, vy) to polar (rho, phi, rho_dot).
  */
  VectorXd NormalizePolar(const VectorXd &z, const VectorXd &z_pred);

	/**
  * normalize the angle between -pi to pi
  */
  VectorXd ToPolar(Eigen::VectorXd & x);

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);  

};

#endif /* TOOLS_H_ */
