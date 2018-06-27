/*
 * EgoVehicle.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#include "EgoVehicle.h"

EgoVehicle::EgoVehicle()
{

}

EgoVehicle::EgoVehicle(double x, double y, double s, double d, double yaw, double spd)
{
  setPose(x, y, s, d, yaw, spd);
}


EgoVehicle::~EgoVehicle()
{
  // TODO Auto-generated destructor stub
}

void EgoVehicle::chooseNextState()
{

}

void EgoVehicle::realizeNextState()
{

}
