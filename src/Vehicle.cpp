/*
 * Vehicle.cpp
 *
 *  The Vehicle class is meant to represent a vehicle that is on the road. It holds
 *  the pose of itself via a Pose data structure, and it knows how to propagate that
 *  pose into the future.
 *
 *  Created on: Jun 22, 2018
 *      Author: deanliu
 */

#include "Vehicle.h"
#include "UtilFunctions.h"
#include <iostream>
#include <math.h>

Vehicle::Vehicle(Track& trk)
:m_rTrack(trk)
{
}

Vehicle::~Vehicle()
{
}

void Vehicle::updatePose(const Pose& p)
{
  m_pose = p;
  float lane_width = 4.0;
  m_pose.lane = (int)(m_pose.d/lane_width);
}


vector<Pose> Vehicle::trajectoryPrediction(float time_horizon)
{
  /* NOTE: each traffic prediction trajectory consists of only 2 poses:
   *      - current pose
   *      - future pose at (time_horizon) ahead
   *      Ego vehicle's path planner only needs that much.
   */

  /* Add traffic lane change behavior prediction here later on, as needed.
   * (check on traffic's velocity vector relative to the Frenet S axis to decide
   *  if traffic is staying or changing lane.)
   */
  return {m_pose, propagatePose(m_pose, time_horizon)};
}


Pose Vehicle::propagatePose(const Pose& p_in, float t)
{
  return propagatePoseFrenet(p_in, t);  // Frenet
//  return propagatePoseInertial(p_in, t);  // Cartesian/Inertial
}


Pose Vehicle::propagatePoseFrenet(const Pose& p_in, float t)
{
  /*
   * Propagate the pose along Frenet coordinate system
   * Assuming as if it's an inertial coord system
   * with no acceleration (it is NOT an inertial coord system.)
   */
  Pose p_new;
    p_new.state       = p_in.state;
    p_new.s           = s_wrap(p_in.s + p_in.spd*t);
    p_new.d           = p_in.d;
    p_new.lane        = p_in.lane; // assume traffic stays in lane
    vector<double> xy = m_rTrack.getXY(p_new.s, p_new.d);
    p_new.x           = xy[0];  // x
    p_new.y           = xy[1];  // y
    p_new.yaw         = xy[2];// yaw
    p_new.spd         = p_in.spd;

  return p_new;
}

Pose Vehicle::propagatePoseInertial(const Pose& p_in, float t)
{
  /*
   * Propagate the pose along Cartesian coordinate system with no acceleration
   */
  Pose p_new;
    p_new.yaw = p_in.yaw;
    p_new.spd = p_in.spd;
    double vx = p_in.spd*cos(p_in.yaw);
    double vy = p_in.spd*sin(p_in.yaw);
    p_new.x = p_in.x + vx*t;
    p_new.y = p_in.y + vy*t;
    vector<double> frenet = m_rTrack.getFrenet(p_new.x, p_new.y, p_new.yaw);
    p_new.s = frenet[0];
    p_new.d = frenet[1];
    p_new.lane = (int)(p_new.d/4.0);

  return p_new;
}
