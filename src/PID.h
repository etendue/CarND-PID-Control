#include <deque>
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
  double p_error_last;

  /*
   * variables for gradient descent algorithms
   */
  std::deque<double> mse;
  std::deque<double> gradient_Kp;
  std::deque<double> gradient_Ki;
  std::deque<double> gradient_Kd;
  double alpha_;
  bool vbose_;

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
   * Reset the error history
   * used for tuning for a new experiment after adjusting the parameters
   */
  void Reset();

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);

  /*
   * Calculate the total PID error.
   * return: the mean square error
   */
  double TotalError();

  /*
   * calculate the GAIN of PID controller
   * param: cte current error between process value and target value
   * param: min minimum value of GAIN
   * param: max maximum value of GAIN
   * param: sample_time used to adjust the PID parameter when sample time or sample rate is changed
   * see reference "Scaling PID tuning parameers for Differet Control sample Rates" @ref www.motioneng.com
   */
  double getControlValue(double cte, const double min = -1.0, const double max =
                             1.0,
                         double sample_time = 1);

  /*
   * function to auto tune the PID using gradient descent
   * @TODO: not working properly, as cost function is not derived properly without knowing Motion Model
   * A placeholder for further development
   */
  void GDAutoTune(double cte);

  /*
   * help function to change the learning rate for gradient descent tuning
   */
  void setLearningRate(double alpha) {
    alpha_ = alpha;
  };

  /*
   * Help function to see debug information during tuning.
   */
  void toggle_vbose() {
    vbose_ = !vbose_;
  };

};

#endif /* PID_H */
