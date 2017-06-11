#include <vector>
#ifndef PID_H
#define PID_H


class PID {
private:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  std::vector<double> squared_err;
  double best_error;
  double dp[3];
  int dp_index;
  int count;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

public:
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

  double getControlValue(double cte);

  bool Twiddle();

  bool needTwiddle();


};

#endif /* PID_H */
