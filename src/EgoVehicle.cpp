/*
 * EgoVehicle.cpp
 *
 *  The EgoVehicle class derives from the Vehicle base class, thus contains all
 *  the base class capabilities. Additionally, the EgoVehicle contains a PathPlanner
 *  object.
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


