#include "PID.h"


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors)
   */
   Kp = Kp_;
   Ki = Ki_i;
   Kd = Kd_;

   p_error = 0;
   i_error = 0;
   d_error = 0;

}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte
   */
   p_error = cte - p_error;
   i_error = cte;
   d_error += cte;

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
