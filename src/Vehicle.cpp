/*
 * Vehicle.cpp
 *
 *  Created on: Jun 22, 2018
 *      Author: deanliu
 */

#include <iostream>
#include <math.h>
//#include "UtilFunctions.h"
#include "Vehicle.h"

Vehicle::Vehicle()
{

}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, double spd)
{
  setPose(x, y, s, d, yaw, spd);
}

Vehicle::~Vehicle()
{

}

void Vehicle::setPose(double x, double y, double s, double d, double yaw, double spd)
{
  m_x = x;
  m_y = y;
  m_s = s;
  m_d = d;
  m_yaw = yaw;
  m_speed = spd;
}

//Vehicle::Coord Vehicle::transformGlobal2Ego(Vehicle::Coord in)
//{
//  Coord out;
//
//  // First translate to vehicle CG
//  double dx = (in.x - m_x);
//  double dy = (in.y - m_y);
//
//  // Then rotate about vehicle yaw (positive counter-clockwise)
//  // https://en.wikipedia.org/wiki/Transformation_matrix#Rotation
//  double cyaw = cos(m_yaw);
//  double syaw = sin(m_yaw);
//  out.x = cyaw*dx + syaw*dy;
//  out.y =-syaw*dx + cyaw*dy;
//
//  return out;
//}
//
//Vehicle::Coord Vehicle::transformEgo2Global(Vehicle::Coord in)
//{
//  Coord out;
//
//  // First rotate about vehicle yaw (positive counter-clockwise)
//  // https://en.wikipedia.org/wiki/Transformation_matrix#Rotation
//  double cyaw = cos(m_yaw);
//  double syaw = sin(m_yaw);
//  double dx = cyaw*in.x - syaw*in.y;
//  double dy = syaw*in.x + cyaw*in.y;
//
//  // then translate from vehicle CG
//  out.x = (m_x + dx);
//  out.y = (m_y + dy);
//
//  return out;
//}
