/*
 * EgoVehicle.h
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef EGOVEHICLE_H_
#define EGOVEHICLE_H_

#include "Vehicle.h"
#include "TrajectoryGenerator.h"

class EgoVehicle : public Vehicle {
 public:
  EgoVehicle(Track& trk);

  virtual ~EgoVehicle();

  TrajectoryGenerator m_TrajGen;

 private:

};

#endif /* EGOVEHICLE_H_ */
