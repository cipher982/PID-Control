#include "PID.h"
#include <iostream>

using namespace std;

// TODO: code here

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  // tau
  this->Kp = Kp;
  this->Kd = Ki;
  this->Ki = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  previous_error = 0.0;
}

void PID::UpdateError(double cte) {

  d_error = cte - p_error;  
  p_error = cte;
  i_error += cte;

}

double PID::TotalError() {
  
  cout << "-" << Kp << " * " << p_error << " - " << Kd << " * " << 
               d_error << " - " << Ki << " * " << i_error << "\tTotalError: " << 
               -Kp * p_error - Kd * d_error - Ki * i_error << endl;

  return - Kp * p_error - Kd * d_error - Ki * i_error;

}

