/*
 * EgoVehicle.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#include "EgoVehicle.h"

EgoVehicle::EgoVehicle(Track& trk)
:Vehicle(trk), m_PathPlanner(*this)
{

}

EgoVehicle::~EgoVehicle()
{

}


