/*
 * TrafficManager.cpp
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

  for (auto itr=traffic_in.begin(); itr!=traffic_in.end(); itr++)
  {
    vector<double> data = *itr;

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
  for (auto itr=m_vehicles.begin(); itr!=m_vehicles.end(); itr++ )
  {
    int id = itr->first;
    vector<Pose> trajectory = itr->second.trajectoryPrediction(m_time_probe);
    m_predictions.insert( {id, trajectory} );
  }
}
