/*
 * TrafficManager.h
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef TRAFFICMANAGER_H_
#define TRAFFICMANAGER_H_

#include <map>
#include <vector>
#include "Track.h"
#include "Vehicle.h"

using std::map;
using std::vector;

class TrafficManager {
 public:
  TrafficManager(Track& trk);
  virtual ~TrafficManager();

  void updateTraffic(vector<vector<double>> traffic_in);

  void predict();

  map<int, Vehicle> m_vehicles;

  // contains predictions of all traffic vehicles
  map<int, vector<Vehicle::Pose>> m_predictions;

 private:

  Track& track;  // reference to the track object for global/frenet conversion services
};

#endif /* TRAFFICMANAGER_H_ */
