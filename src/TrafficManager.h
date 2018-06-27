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
#include "Vehicle.h"

using std::map;
using std::vector;

class TrafficManager {
 public:
  TrafficManager();
  virtual ~TrafficManager();

  void update_traffic(vector<vector<double>> traffic_in);

  void predict();

 private:
  map<int, Vehicle> m_vehicles;

};

#endif /* TRAFFICMANAGER_H_ */
