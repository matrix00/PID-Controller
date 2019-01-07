#include "PID.h"

#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() 
{
  best_err = 10000.0;//std::numeric_limits<double>::max();
	converged = false;
	
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p[0] = Kp;
	p[1] = Kd;
	p[2] = Ki;

	p_error = 0;
	d_error = 0;
	i_error = 0;
}

void PID::UpdateError(double cte) {

	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

//	errQ.push(cte);

//	if (errQ.size() > 6)
//		updateParamWithTwiddle();
			
}

double PID::TotalError() {
	return (p_error + d_error + i_error);
}

void PID::updateParamWithTwiddle()
{
	
	int it=0;

		for (int i=0; i < 3; i++)
		{
			p[i] += dp[i];
			// need to do something x_trajectory, y_trajectory, err = run(robot, p)

			double err = errQ.front();// need to find out
			errQ.pop();
			if (err < best_err)
     	{
				best_err = err;
				dp[i] *= 1.1;
			}
			else
			{
				p[i] -= 2*dp[i];
				//x_trajectory, y_trajectory, err = run(robot, p)
				err = errQ.front();
				errQ.pop();
        if (err < best_err)
				{
					best_err = err;
					dp[i] *= 1.1;
				}
				else
				{
					p[i] += dp[i];
					dp[i] *= 0.9;
				}
			}
			it +=1;
		}
	double dpTotal = dp[0]+dp[1]+dp[2];
	//if (dpTotal < 0.0001)
	//{
		Kp = p[0];
		Kd = p[1];
		Ki = p[2]; 
		converged = true;
	//}
	std::cout<<" Kp " << Kp << " Kd " << Kd << " Ki " << Ki << " best err " << best_err<<  " dp total " << dpTotal << " P0 " << p[0] << " P1 " << p[1] << " P2 " << p[2] << endl;
}
