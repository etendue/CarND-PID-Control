/*
 * Car.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author: etendue
 */

#include <math.h>
#include <algorithm>
#include <iostream>
#include "Car.h"
using namespace std;

static const double max_steering_angle = M_PI/4;

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
  orientation_=fmod(orientation,2*M_PI);
}

void Car::move(double distance, double steering, const double tolerance) {
	//check steering angle boundary
	double steering_abs = min(fabs(steering),max_steering_angle);
	steering = copysign(steering_abs,steering);

	//check the distance boundary
	distance = max(0.0,distance);

	//add noise if necessary
	//the turn angle based on bicycle mode
	double turn_angle = tan(steering) * distance / length_;

	if (fabs(turn_angle) < tolerance) {
		// approximate by straight line motion
		x_ += distance * cos(orientation_);
		y_ += distance * sin(orientation_);
	} else {
		// approximate bicycle model for motion
		double radius = distance / turn_angle;
		x_ += radius * (sin(orientation_ + turn_angle) - sin(orientation_));
		y_ += radius * (cos(orientation_ + turn_angle) - cos(orientation_));
	}
	orientation_ = fmod(orientation_ + turn_angle, 2.0 * M_PI);

}
double Car::run(Car& car, double params[],int steps,double speed){

   double err = 0.0;
   double prev_cte = car.y_;
   double integ_cte = prev_cte;
   double cte = 0;

   for(uint i=0; i < steps; i++){
     cte= car.y_;
     double cte_d = cte - prev_cte;
     integ_cte += cte;
     double steer = -params[0] * cte - params[1]*cte_d - params[2]*integ_cte;
     car.move(speed,steer);
     //cout <<"x:" <<car.x_ <<" y:"<<car.y_<<endl;
     prev_cte = cte;
     err += cte*cte;
   }
   return err;
}
