#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_){
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	iteration = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
	if (iteration == 0) {
		prev_cte = cte;
	}
	p_error = cte;
	i_error += cte;
	d_error = cte - prev_cte;
	prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */


  return 0.0;  // TODO: Add your total error calc here!
}