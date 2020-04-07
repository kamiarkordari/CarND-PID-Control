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
   dp[0] = 1;
   dp[1] = 1;
   dp[2] = 1;

   parameter_tuning_index = 0;
   error_accumulation_started = false;
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
   int P[3];
   P[0] = this->Kp;
   P[1] = this->Ki;
   P[2] = this->Kd;

   if (dp[0]+dp[1]+dp[2] > tol) {
     if (state == "0"){
       P[parameter_tuning_index] += dp[parameter_tuning_index];
       Kp = P[0];
       Ki = P[1];
       Kd = P[2];
       state = "1";
       counter = 0;
       error = 0;
       error_accumulation_started = true;
       return;
     }
     if (state == "1" && error_accumulation_stopped){
       if (error < best_error) {
         best_error = error;
         dp[parameter_tuning_index] *= 1.1;
       } else {
         P[parameter_tuning_index] -= 2*dp[parameter_tuning_index];
         this->Kp = P[0];
         this->Ki = P[1];
         this->Kd = P[2];
         state = "2";
         counter = 0;
         error = 0;
         error_accumulation_started = true;
         return;
       }
     }

     if (state == "2" && error_accumulation_stopped){
       if (error < best_error) {
         best_error = error;
         dp[parameter_tuning_index] *= 1.1;
       } else {
         P[parameter_tuning_index] += dp[parameter_tuning_index];
         this->Kp = P[0];
         this->Ki = P[1];
         this->Kd = P[2];
         dp[parameter_tuning_index] *= 0.9;
       }
       state = "0";
     }
   }
}

void PID::TwiddleError(double cte) {
  if (counter >= n && error_accumulation_started) {
    error += cte*cte;
  }
  if (counter == 2*n && error_accumulation_started) {
    error_accumulation_started = false;
    error_accumulation_stopped = true;
  }
}
