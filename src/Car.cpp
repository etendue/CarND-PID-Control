/*
 * Car.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author: etendue
 */

#include <math.h>
#include "Car.h"


Car::Car(double length) {
  x_=0;
  y_=0;
  orientation_=0;
  length_= length;
}

Car::~Car() {
}

void Car::init(double x, double y, double orientation) {
  x_=x;
  y_=y;
  orientation_=orientation % (2*M_PI);
}

void Car::move(double speed, double steering) {
}
