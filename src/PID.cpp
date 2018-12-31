#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

void PID::UpdateError(double cte) {

	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;


}

double PID::TotalError() {
	return (p_error + d_error + i_error);
}

