/*
 * TrafficManager.cpp
 *
 *  The TrafficManager class holds knowledge about all the known (sensed) traffic
 *  vehicles on the track, as well as predictions of each vehicle's future pose.
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#include "TrafficManager.h"
#include "UtilFunctions.h"
#include <iostream>
#include <vector>
#include <math.h>

using std::vector;
using std::cout;
using std::endl;

TrafficManager::TrafficManager(Track& trk)
:m_rTrack(trk)
{
}

TrafficManager::~TrafficManager()
{
}

void TrafficManager::updateTraffic(const vector<vector<double>>& traffic_in)
{
  m_vehicles.clear(); // clear out past data

  for (const auto& data : traffic_in)
  {
    int id = (int)data[0];
    Pose p;
      p.x =       data[1];
      p.y =       data[2];
      double vx = data[3];
      double vy = data[4];
      p.s =       data[5];
      p.d =       data[6];
      p.lane =    (int)(p.d/4.0); // each lane is 4m wide
      p.yaw =     atan2(vy, vx);
      p.spd =     magnitude(vx, vy);
    Vehicle vehicle(m_rTrack);
    vehicle.updatePose(p);
    m_vehicles.insert( {id, vehicle} );
  }
}

void TrafficManager::predict()
{
  m_predictions.clear();

  // get all traffic vehicles to create prediction of itself into the future
  for (auto& kv : m_vehicles)
  {
    int id = kv.first;
    m_predictions.insert( {id, kv.second.trajectoryPrediction(m_time_probe)});
  }
}
