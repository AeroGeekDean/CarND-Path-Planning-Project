/*
 * EgoVehicle.h
 *
 *  The EgoVehicle class derives from the Vehicle base class, thus contains all
 *  the base class capabilities. Additionally, the EgoVehicle contains a PathPlanner
 *  object.
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef EGOVEHICLE_H_
#define EGOVEHICLE_H_

#include "Vehicle.h"
#include "PathPlanner.h"

class EgoVehicle : public Vehicle {
 public:
  EgoVehicle(Track& trk);

  virtual ~EgoVehicle();

  PathPlanner m_PathPlanner;

 private:

};

#endif /* EGOVEHICLE_H_ */
