/*
 * TrafficManager.h
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef TRAFFICMANAGER_H_
#define TRAFFICMANAGER_H_

#include "Vehicle.h"
#include "Track.h"
#include <map>
#include <vector>

using std::map;
using std::vector;

class TrafficManager {
 public:
  TrafficManager(Track& trk);
  virtual ~TrafficManager();

  void updateTraffic(const vector<vector<double>>& traffic_in);

  void predict();

  float m_time_probe; // [sec], look ahead time horizon used to probe traffic trajectory

  map<int, Vehicle> m_vehicles;

  // contains predictions of all traffic vehicles
  map<int, vector<Pose>> m_predictions;

 private:

  Track& m_rTrack;  // reference to the track object for global/frenet conversion services
};

#endif /* TRAFFICMANAGER_H_ */
