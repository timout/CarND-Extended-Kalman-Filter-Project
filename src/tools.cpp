#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools {

    VectorXd NormalizePolar(const VectorXd &z, const VectorXd &z_pred)
    {
        VectorXd y = z - z_pred;
        while( y(1) > M_PI ) y(1) -= M_PI * 2;
        while( y(1) < -M_PI ) y(1) += M_PI * 2;
        return y;
    }

    VectorXd ToPolar(Eigen::VectorXd & x)
    {
      float px = x[0];
      float py = x[1];
      float vx = x[2];
      float vy = x[3];
      float rho = sqrt(px*px + py*py);
      if ( rho < EPSILON ) rho = EPSILON;
      float phi = atan2(py, px);
      float rho_dot = (px * vx + py * vy) / rho;

      VectorXd z = VectorXd(3);
      z << rho, phi, rho_dot;

      return z;
    }

    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) 
    {
      VectorXd rmse(4);
      rmse << 0,0,0,0;
      
      if ( estimations.size() == 0 || estimations.size() != ground_truth.size()) {
          std::cout << "Invalid estimation or ground_truth data" << std::endl;
          return rmse;
      }
      
      //accumulate squared residuals
      for(unsigned int i=0; i < estimations.size(); ++i){
          VectorXd residual = estimations[i] - ground_truth[i];
          //coefficient-wise multiplication
          residual = residual.array() * residual.array();
          rmse += residual;
      }
      //mean
      rmse = rmse/estimations.size();
      //squared root
      rmse = rmse.array().sqrt();
      return rmse;
    }

    MatrixXd CalculateJacobian(const VectorXd& x_state) {

      MatrixXd Hj(3,4);
      //recover state parameters
      float px = x_state(0);
      float py = x_state(1);
      float vx = x_state(2);
      float vy = x_state(3);
    
      //pre-compute a set of terms to avoid repeated calculation
      float c1 = px*px + py*py;
      float c2 = sqrt(c1);
      float c3 = (c1 * c2);
    
      //check division by zero
      if( fabs(c1) < 0.0001 ){
          std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
          return Hj;
      }
    
      //compute the Jacobian matrix
      Hj << (px/c2), (py/c2), 0, 0,
          -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
      
      return Hj;
    }

};
