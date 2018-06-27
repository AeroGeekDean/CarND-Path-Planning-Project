/*
 * EgoVehicle.h
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef EGOVEHICLE_H_
#define EGOVEHICLE_H_

#include "Vehicle.h"

class EgoVehicle : public Vehicle {
 public:
  EgoVehicle();
  EgoVehicle(double x, double y, double s, double d, double yaw, double spd);

  virtual ~EgoVehicle();

  void chooseNextState();
  void realizeNextState();


 private:

};

#endif /* EGOVEHICLE_H_ */
