#include "PID.h"
#include <iostream>
#include <algorithm>
#include <numeric>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  p_error_last = 0;
  Kp = 0;
  Ki = 0;
  Kd = 0;
  alpha_ = 0.0001;
  vbose_ = false;
}

PID::~PID() {
}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  Reset();
}

void PID::Reset() {
  p_error = 0;
  i_error = 0;
  d_error = 0;
  p_error_last = 0;
  mse.clear();
  gradient_Kd.clear();
  gradient_Ki.clear();
  gradient_Kd.clear();
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  d_error = p_error - p_error_last;
  p_error_last = p_error;

  //caching the squared error, keeping a queue for 50 elements
  mse.push_front(cte * cte);
  if (mse.size() > 50) {
    mse.pop_back();
  }
}

double PID::TotalError() {
  if (mse.size() > 0)
    return accumulate(mse.begin(), mse.end(), 0.0) / mse.size();
  return 0;
}

double PID::getControlValue(double cte, const double min, const double max,
                            double sample_time) {

  UpdateError(cte);

  //check numeric stability
  if (sample_time == 0) {
    sample_time = 0.0001;
  }
  //calculate gain
  double gain = -Kp * p_error - Kd * d_error * sample_time
      - Ki * i_error / sample_time;

  //cut the boundary
  if (gain < min) {
    gain = min;
  }
  if (gain > max) {
    gain = max;
  }
  return gain;
}

void PID::GDAutoTune(double cte) {

  //the d_cost /d_K = f(gain) * error_type *  d_error/dt
  // cost function = mse over last N sample points.
  // f(gain): transfer function of motion model = current cte
  // error_type: p_error | i_error | d_error

  //TODO: Not working properly.

  gradient_Kp.push_back(-cte * p_error * d_error);
  gradient_Ki.push_back(-cte * i_error * d_error);
  gradient_Kd.push_back(-cte * d_error * d_error);

  if (gradient_Kp.size() > 10)
    gradient_Kp.pop_front();
  if (gradient_Ki.size() > 10)
    gradient_Ki.pop_front();
  if (gradient_Kd.size() > 10)
    gradient_Kd.pop_front();

  double dKp = accumulate(gradient_Kp.begin(), gradient_Kp.end(), 0.0)
      / gradient_Kp.size();
  double dKi = accumulate(gradient_Ki.begin(), gradient_Ki.end(), 0.0)
      / gradient_Ki.size();
  double dKd = accumulate(gradient_Kd.begin(), gradient_Kd.end(), 0.0)
      / gradient_Kd.size();

  Kp -= alpha_ * dKp;
  //Ki -= alpha_ * dKi;
  Kd -= alpha_ * dKd;

  if (vbose_) {
    cout << "Kp, Ki, Kd, dKp, dKi,dKd:" << Kp << " " << Ki << " " << Kd << " "
         << dKp << " " << dKi << " " << dKd << endl;
  }
}
