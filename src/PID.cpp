#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors)
   */
   this->Kp = Kp_;
   this->Ki = Ki_;
   this->Kd = Kd_;

   p_error = 0;
   i_error = 0;
   d_error = 0;

   tol = 0.1;
   dp[0] = 0.1; // 0.1
   dp[1] = 0.0001; //0.001
   dp[2] = 1; //1

   parameter_tuning_index = 0;
   counter = -update_cycle;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte
   */
   d_error = cte - p_error;
   p_error = cte;
   i_error += cte;
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  double output = -Kp * p_error - Kd * d_error - Ki * i_error;

  if (output < -1) {
    output = -1;
  }

  if (output > 1) {
    output = 1;
  }

  return output;
}


void PID::Twiddle(double cte) {
  /**
   * Tune PID paramters to minimze the error
   */
   double P[3];
   P[0] = Kp;
   P[1] = Ki;
   P[2] = Kd;

  // Accumulate error
   error += cte*cte;

   // Calculation best error. Note: counter starts at -update_cycle
   if (counter == -1) {
     best_error = error/update_cycle;
   }
   else if (counter == 0) {
     if (dp[0]+dp[1]+dp[2] > tol) {
       // Vary one parameter at a time
       P[parameter_tuning_index] += dp[parameter_tuning_index];
       Kp = P[0];
       Ki = P[1];
       Kd = P[2];
       error = 0;
     }
   }
   else if (counter == update_cycle) {
     error /= update_cycle;
     // Measure the resulting error difference in error
     // - if increasing the value has improved the overall error keep the change
     // - Increase dp for that parameter to increase the search range for the optimal value
     if (error < best_error) {
       best_error = error;
       error = 0;
       dp[parameter_tuning_index] *= 1.1;
       counter = -1;
       parameter_tuning_index++;
       parameter_tuning_index = parameter_tuning_index % 3;
     } else {
       // If increasing the PID parameter value has inreased the overall error change the paramter in the other direction
       P[parameter_tuning_index] -= 2*dp[parameter_tuning_index];
       Kp = P[0];
       Ki = P[1];
       Kd = P[2];
       error = 0;
     }
   }
   else if (counter == 2*update_cycle) {
     error /= update_cycle;
     counter = -1;

     // If decreasing the PID parameter value has decreased the overall error,
     // increase dp for that parameter to increase the search range for the optimal value
     if (error < best_error) {
       best_error = error;
       error = 0;
       dp[parameter_tuning_index] *= 1.1;
     } else {
     // If decreasing the PID parameter value has increased the overall error,
     // decrease dp for that parameter to decrease the search range for the optimal value
       P[parameter_tuning_index] += dp[parameter_tuning_index];
       dp[parameter_tuning_index] *= 0.9;
       Kp = P[0];
       Ki = P[1];
       Kd = P[2];
       parameter_tuning_index++;
       parameter_tuning_index = parameter_tuning_index % 3;
       error = 0;
     }
   }
   counter++;
}
