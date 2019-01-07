#ifndef PID_H
#define PID_H

#include <queue>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

	double dp[3] = {1, 1, 1};
  double p[3];
  double best_err;// = std::numeric_limits<double>::max();
	std::queue<double> errQ; 

	bool converged;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

	//update co-efficient using Twiddle logic
	void updateParamWithTwiddle();
};

#endif /* PID_H */
