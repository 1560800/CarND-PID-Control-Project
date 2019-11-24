#include "PID.h"
#include <iostream>
#include <limits>
#include <cmath>
using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_){

	Kp = Kp_;
	Ki = Ki_;
	Kd = Kd_;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	iteration = 0;

	// for anti windup
	max_int_cte = 100000.0;

	twiddle_enable = false;
	steps = 0;
	count_twiddle = -2;
	twiddle_err = 0.0;
	twiddle_best_err = numeric_limits<double>::max();
	pid_param = { Kp, Ki, Kd };
	dp = { 0.1 * Kp, 0.1 * Ki, 0.1 * Kd };
	dp_param_index = 0;
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
	++iteration;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
	double total_error = Kp * p_error + Ki * i_error + Kd * d_error;
	if (total_error > 1.0)
		total_error = 1.0;
	if (total_error < -1.0)
		total_error = -1.0;
	return total_error;// TODO: Add your total error calc here!
}

void PID::Twiddle(double cte) {
	twiddle_err += pow(cte, 2);

	if ((steps % 500) == 0) {
		if (count_twiddle < 0) {
			count_twiddle += 1;
			twiddle_err = 0;
		}
		else if (count_twiddle == 0) {
			// update best
			twiddle_best_err = twiddle_err;
			// reset error
			twiddle_err = 0;
			// update hyper param
			UpdateKpid(dp_param_index, dp[dp_param_index]);
			// set for next loop
			count_twiddle = 1;

		}
		else if (count_twiddle == 1) {
			if (twiddle_err < twiddle_best_err) {
				// update best
				twiddle_best_err = twiddle_err;
				// reset error
				twiddle_err = 0;
				// update dp
				dp[dp_param_index] *= 1.1;

				// got to next param
				steps = 0;
				count_twiddle = 1;
				dp_param_index = (dp_param_index + 1) % 3;
				UpdateKpid(dp_param_index, dp[dp_param_index]);
			}
			else {
				UpdateKpid(dp_param_index, (-2.0 * dp[dp_param_index]));
				count_twiddle = 2;
				// reset error
				twiddle_err = 0;
			}
		}
		else if (count_twiddle == 2) {
			if (twiddle_err < twiddle_best_err) {
				// update best
				twiddle_best_err = twiddle_err;
				dp[dp_param_index] *= 1.1;
			}
			else {
				UpdateKpid(dp_param_index, dp[dp_param_index]);
				dp[dp_param_index] *= 0.9;
			}
			// reset error
			twiddle_err = 0;
			// got to next param
			steps = 0;
			count_twiddle = 1;
			dp_param_index = (dp_param_index + 1) % 3;
			UpdateKpid(dp_param_index, dp[dp_param_index]);
		}
	}
	steps++;
	std::cout << "steps : " << steps << std::endl;
	std::cout << "count_twiddle : " << count_twiddle << std::endl;
	std::cout << "param index : " << dp_param_index << std::endl;
	std::cout << "err : " << twiddle_err << std::endl;
	std::cout << "best err : " << twiddle_best_err << std::endl;
	std::cout << "PID value : " << Kp << ":" << Ki << ":" << Kd << std::endl;
}

void PID::UpdateKpid(int index, double value) {
	switch (index) {
	case 0:
		Kp += value;
	case 1:
		Ki += value;
	case 2:
		Kd += value;
	}
	std::cout << "Kp : " << Kp << "/ Ki : " << Ki << "/ Kd : " << Kd << std::endl;
}