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
  p_error_last = 0;
  Kp =0;
  Ki =0;
  Kd =0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error=0;
  i_error=0;
  d_error=0;
  p_error_last = 0;
}

void PID::UpdateError(double cte) {
  p_error = cte;
  i_error += cte;
  d_error = p_error - p_error_last;
  p_error_last = p_error;
}

double PID::TotalError() {
 return 0;
}
bool PID::TuneKp(){

//	static bool second_try = false;
//	if(squared_err.size()>=100){
//		double mse = accumulate(squared_err.begin()+50, squared_err.end(), 0.0)/(squared_err.size()-50);
//		squared_err.clear();
//		count = 0;
//		if(mse < 0.005){
//			best_Kp = Kp;
//			cout <<"Tune Kp finished";
//			cout<<"New best mse:"<<best_mse<<" with Kp:"<<best_Kp<<endl;
//			return false; // Tune finished
//		}
//
//		if(mse < best_mse){
//			best_mse = mse;
//			best_Kp = Kp;
//			dp[0] *= 1.1;
//			Kp = best_Kp + dp[0];
//			second_try=false;
//			cout<<"New best mse:"<<best_mse<<" with Kp:"<<best_Kp<<endl;
//		}else if(second_try==false)
//		{
//			Kp = best_Kp - dp[0];
//			second_try = true;
//		}else{
//			dp[0] *=0.9;
//			Kp = best_Kp + dp[0];
//			second_try = false;
//		}
//		cout<<"New mse is:"<<mse<<" best_mse is:"<<best_mse<<" with Kp:"<<best_Kp<<endl;
//		cout<<"Dp is now:"<<dp[0]<<" Kp:"<<Kp<<endl;
//		return true;
//	}
	return false;

}

double PID::getControlValue(double cte, const double min, const double max){

  UpdateError(cte);
  double gain =  -Kp *p_error-Kd * d_error -Ki * i_error;
  if (gain <min){
	  gain = min;
  }
  if (gain > max){
	  gain = max;
  }
  return gain;
}
