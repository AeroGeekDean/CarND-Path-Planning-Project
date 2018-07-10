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
  float lane_width = 4.0;
  m_pose.lane = (int)(m_pose.d/lane_width);
  m_lane = m_pose.lane;
}

void Vehicle::updateState(const string& state_in)
{
  m_pose.state = state_in;
}

Pose Vehicle::propagatePose(const Pose& p_init, float t)
{
  return propagatePoseFrenet(p_init, t);
//  return propagatePoseInertial(p_init, t);
}


Pose Vehicle::propagatePoseFrenet(const Pose& p_init, float t)
{
  /*
   * Propagate the pose along Frenet coordinate system
   * (assuming as if it's an inertial coord system)
   * with no acceleration
   */
  Pose p_new;
    p_new.state = p_init.state;
    p_new.s = p_init.s + p_init.spd*t;
    p_new.d = p_init.d;
    p_new.lane = p_init.lane; // assume traffic stays in lane
    vector<double> xy = m_rTrack.getXY(p_new.s, p_new.d);
    p_new.x = xy[0];  // x
    p_new.y = xy[1];  // y
    p_new.yaw = xy[2];// yaw
    p_new.spd = p_init.spd;
    p_new.vx = p_new.spd*cos(p_new.yaw);
    p_new.vy = p_new.spd*sin(p_new.yaw);

    return p_new;
}

Pose Vehicle::propagatePoseInertial(const Pose& p_init, float t)
{
  /*
   * Propagate the pose along Cartesian coordinate system with no acceleration
   */
  Pose p_new;
    p_new.x = p_init.x + p_init.vx*t;
    p_new.y = p_init.y + p_init.vy*t;
    p_new.vx = p_init.vx; // currently, accel = 0
    p_new.vy = p_init.vy;

    p_new.yaw = atan2(p_init.vy, p_init.vx);
    p_new.spd = p_init.spd; //magnitude(p_init.vx, p_init.vy);
    vector<double> frenet = m_rTrack.getFrenet(p_new.x, p_new.y, p_new.yaw);
    p_new.s = frenet[0];
    p_new.d = frenet[1];

  return p_new;
}


vector<Pose> Vehicle::trajectoryPrediction(float time_horizon)
{
  /* NOTE: each traffic prediction trajectory consists of only 2 poses:
   *      - current pose
   *      - pose at (time_horizon) ahead in future
   *      Ego vehicle's path planner only needs that much.
   */

  vector<Pose> trajectory;
  trajectory.push_back(m_pose);

  /* Add traffic lane change behavior prediction here later on, as needed.
   * (check on traffic's velocity vector relative to the Frenet S axis to decide)
   */

  trajectory.push_back(propagatePose(m_pose, time_horizon));

  return trajectory;
}
