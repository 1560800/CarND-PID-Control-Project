#ifndef PID_H
#define PID_H
#include <vector>

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

  // Twiddle
  void Twiddle(double cte);
  void UpdateKpid(int index, double value);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  double total_cte;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

  int iteration;
  double prev_cte;
  double max_int_cte;

 /**
 * twiddle variables
 */
  bool twiddle_enable;
  int steps;
  int count_twiddle;
  double twiddle_err;
  double twiddle_best_err;
  std::vector<double> dp;
  std::vector<double> pid_param;
  int dp_param_index;


};

#endif  // PID_H