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
   dp[0] = 0.1;
   dp[1] = 0.0001;
   dp[2] = 1;

   parameter_tuning_index = 0;
   counter = -wait_cycles;
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
   P[0] = this->Kp;
   P[1] = this->Ki;
   P[2] = this->Kd;

   error += cte*cte;

   if (counter == -1) {
     best_error = error/wait_cycles;
   }
   else if (counter == 0) {
     if (dp[0]+dp[1]+dp[2] > tol) {
       P[parameter_tuning_index] += dp[parameter_tuning_index];
       this->Kp = P[0];
       this->Ki = P[1];
       this->Kd = P[2];
       error = 0;
     }
   }
   else if (counter == wait_cycles) {
     error /= wait_cycles;
     if (error < best_error) {
       best_error = error;
       error = 0;
       dp[parameter_tuning_index] *= 1.1;
       counter = -1;
       parameter_tuning_index++;
       parameter_tuning_index = parameter_tuning_index % 3;
     } else {
       P[parameter_tuning_index] -= 2*dp[parameter_tuning_index];
       this->Kp = P[0];
       this->Ki = P[1];
       this->Kd = P[2];
       error = 0;
     }
   }
   else if (counter == 2*wait_cycles) {
     error /= n;
     counter = -1;

     if (error < best_error) {
       best_error = error;
       error = 0;
       dp[parameter_tuning_index] *= 1.1;
     } else {
       P[parameter_tuning_index] += dp[parameter_tuning_index];
       dp[parameter_tuning_index] *= 0.9;
       this->Kp = P[0];
       this->Ki = P[1];
       this->Kd = P[2];
       parameter_tuning_index++;
       parameter_tuning_index = parameter_tuning_index % 3;
       error = 0;
     }
   }
   counter++;
}
