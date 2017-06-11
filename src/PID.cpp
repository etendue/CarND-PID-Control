#include "PID.h"
#include "Car.h"
#include <iostream>
#include <algorithm>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  p_error=0;
  i_error=0;
  d_error=0;
  Kp =0;
  Ki =0;
  Kd =0;
  count =0;
  //for twindle
  best_error=numeric_limits<double>::max();
  dp[0] = 0.1;
  dp[1] = 0.01;
  dp[2] = 0.01;
  dp_index = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  count = 0;
  p_error=0;
  i_error=0;
  d_error=0;
}

void PID::UpdateError(double cte) {
  if(count == 0){
    d_error = 0;
  }else
  {
    d_error = cte-p_error;
  }
  count++;
  p_error = cte;
  i_error += cte;

  if(count >100){
    squared_err.push_back(cte*cte);
  }

}

double PID::TotalError() {
  if(squared_err.size() > 0)
  {
    double sum = accumulate(squared_err.begin(), squared_err.end(), 0.0);
    return sum / squared_err.size();
  }else
  {
    return 0;
  }
}

double PID::getControlValue(double cte){

  UpdateError(cte);
  return -Kp *p_error-Kd * d_error -Ki * i_error;
}

bool PID::needTwiddle(){
  return squared_err.size()>=100;
}

bool PID::Twiddle(){
  static double p_temp = 0;
  static bool addition = true;
  double p[3] = { Kp, Ki, Kd };

  //if ((fabs(dp[0]) + fabs(dp[1]) + fabs(dp[2])) > 0.0001) {
    double err = TotalError();
    squared_err.clear();
    if(err < best_error)
    {
      best_error = err;
      cout<<"New best err:"<<best_error;
      cout <<" with parameter: "<<"Kp:"<<Kp<<" Kd:"<<Kd<<" Ki:"<<Ki<<endl;

      p_temp = p[dp_index];
      dp[dp_index] = dp[dp_index] * 1.2;
      p[dp_index] = p_temp + dp[dp_index];
      addition = true;
    }else if(addition)
    {
      p[dp_index] = p_temp - dp[dp_index];
      cout << p[dp_index]<<endl;
      addition = false;
    }else if(dp[dp_index]>0.01){
      dp[dp_index] *= 0.8;
      p[dp_index] = p_temp + dp[dp_index];
      addition = true;
    }else{
      p[dp_index] = p_temp;
      dp[dp_index] = 0.1;
      dp_index = (dp_index +1)%3;
      p_temp = p[dp_index];
      p[dp_index] += dp[dp_index];
      addition = true;
    }

    Init(p[0], p[1], p[2]);
    cout <<" New parameter: "<<"Kp:"<<Kp<<" Kd:"<<Kd<<" Ki:"<<Ki<<endl;
    cout <<" dp: "<<dp[0]<<" "<<dp[2]<<" "<<dp[1]<<endl;
    return false;
 /* }else
  {
    squared_err.clear();
    cout <<" Optimization finished: "<<endl;
    cout <<"Kp:"<<Kp<<" Kd:"<<Kd<<" Ki:"<<Ki<<endl;
    return true;
  }*/

}

