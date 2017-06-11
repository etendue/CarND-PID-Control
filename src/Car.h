/*
 * Car.h
 *
 *  Created on: Jun 8, 2017
 *      Author: etendue
 */
#include <vector>

#ifndef CAR_H_
#define CAR_H_

class Car {
 public:
  Car(double length=2.6);
  virtual ~Car();
  void init(double x,double y, double orientation);
  void move(double distance,double steering,const double tolerance=0.001);
  static double run(Car& car,double params[],int steps,double speed);

  double x_;
  double y_;
  double orientation_;
private:
  double length_;
};

#endif /* CAR_H_ */
