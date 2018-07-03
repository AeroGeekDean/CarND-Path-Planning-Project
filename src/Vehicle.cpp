/*
 * Vehicle.cpp
 *
 *  Created on: Jun 22, 2018
 *      Author: deanliu
 */

#include "Vehicle.h"
#include <iostream>
#include <math.h>
#include "UtilFunctions.h"

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

  double lane_width = 4.0;
  m_lane = (int)(m_pose.d/lane_width);
}

Pose Vehicle::propagatePose(float t)
{
  return propagatePoseFrenet(t);
//  return propagatePoseInertial(t);
}


Pose Vehicle::propagatePoseFrenet(float t)
{
  Pose p_new;
    p_new.s = m_pose.s + m_pose.spd*t;
    p_new.d = m_pose.d;
    vector<double> xy = m_rTrack.getXY(p_new.s, p_new.d);
    p_new.x = xy[0];  // x
    p_new.y = xy[1];  // y
    p_new.yaw = xy[2];// yaw
    p_new.spd = m_pose.spd;
    p_new.vx = p_new.spd*cos(p_new.yaw);
    p_new.vy = p_new.spd*cos(p_new.yaw);

    return p_new;
}

Pose Vehicle::propagatePoseInertial(float t)
{
  Pose p_new;
    p_new.x = m_pose.x + m_pose.vx*t;
    p_new.y = m_pose.y + m_pose.vy*t;
    p_new.vx = m_pose.vx; // currently, accel = 0
    p_new.vy = m_pose.vy;

    p_new.yaw = atan2(m_pose.vy, m_pose.vx);
    p_new.spd = m_pose.spd; //magnitude(m_pose.vx, m_pose.vy);
    vector<double> frenet = m_rTrack.getFrenet(p_new.x, p_new.y, p_new.yaw);
    p_new.s = frenet[0];
    p_new.d = frenet[1];

  return p_new;
}


vector<Pose> Vehicle::trajectoryPrediction(float time_horizon, float dt)
{
  vector<Pose> trajectory;

  for (float t=0.0; t<=time_horizon; t+=dt)
  {
    trajectory.push_back(propagatePose(t));

    /* Question: Should we incremental fwd-in-time pose be computed from its prev pose?
     *           Or just computed directly forward from current (time=0) pose?
     *     Note: If accelerations are zero or constant, then the answers are same.
     */
  }

  return trajectory;
}
