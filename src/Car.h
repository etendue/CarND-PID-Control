/*
 * Car.h
 *
 *  Created on: Jun 8, 2017
 *      Author: etendue
 */

#ifndef CAR_H_
#define CAR_H_

class Car {
 public:
  Car(double length=4);
  virtual ~Car();
  void init(double x,double y, double orientation);
  void move(double speed,double steering);
 private:
  double x_;
  double y_;
  double orientation_;
  double length_;
};

#endif /* CAR_H_ */
