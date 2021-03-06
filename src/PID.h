#ifndef PID_H
#define PID_H

#include <string>
using namespace std;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Tune PID paramters
   * @param cte The current cross track error
   */
  void Twiddle(double cte);


// private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /**
   * Twiddle Paramters
   */
  double dp[3];
  float tol;
  double best_error;
  double error;
  const int update_cycle = 1000;
  int counter;
  int parameter_tuning_index;
};

#endif  // PID_H
