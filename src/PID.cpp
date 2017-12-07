#include "PID.h"

using namespace std;

// TODO: code here

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  // tau
  this->Kp = Kp;
  this->Kd = Ki;
  this->Ki = Kd;

  p_error = 0; // previous error
  d_error = 0; // new error
  i_error = 0; // iterate this
}

void PID::UpdateError(double cte) {

  /* PYTHON CODE

  diff_cte = cte - prev_cte
  prev_cte = cte
  int_cte += cte
  steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte
  */

  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;


}

double PID::TotalError() {
  
  //d_error = cte - p_error;
  //p_error = cte;
  //i_error += cte;

  // steer = -tau_p * cte - tau_d * diff_cte - tau_i * int_cte // Python version
  double turn = (-Kp * p_error) + (-Kd * d_error) + (-Ki * i_error);
  return turn;

}

