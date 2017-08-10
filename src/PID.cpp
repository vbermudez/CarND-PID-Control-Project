#include <limits>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp; // Proportional
  this->Ki = Ki; // Integral
  this->Kd = Kd; // Differential

  // Assuming the car is far from the CTE.
  this->p_error = numeric_limits<double>::max(); // prev cte
  // At the very beginning, the total area between the position of ther car and the CTE is null.
  this->i_error = 0.0; // I cte
  // Assuming the position of the car is far from CTE, at the begining.
  this->d_error = 0.0; // D cte
}

void PID::UpdateError(double cte) {
  if (this->p_error == numeric_limits<double>::max()) {
    this->p_error = cte;
  }

  this->d_error = cte - this->p_error;
  this->p_error = cte;
  this->i_error += cte;
}

double PID::TotalError() {
  // Using -tau_p * CTE - tau_d * diff_CTE - tau_i * int_CTE from lesson.
  return -this->Kp * this->p_error - this->Kd * this->d_error - this->Ki * this->i_error;
}

