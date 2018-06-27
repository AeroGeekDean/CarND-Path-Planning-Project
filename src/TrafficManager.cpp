/*
 * TrafficManager.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#include "TrafficManager.h"
#include <iostream>
#include <vector>
#include <math.h>
#include "UtilFunctions.h"

using std::vector;
using std::cout;
using std::endl;

TrafficManager::TrafficManager()
{

}

TrafficManager::~TrafficManager()
{

}

void TrafficManager::update_traffic(vector<vector<double>> traffic_in)
{
  m_vehicles.clear(); // clear out past data

  for (auto itr = traffic_in.begin(); itr!=traffic_in.end(); itr++)
  {
    vector<double> traffic_data = *itr;

    int id = (int)traffic_data[0];
    double x =    traffic_data[1];
    double y =    traffic_data[2];
    double vx =   traffic_data[3];
    double vy =   traffic_data[4];
    double s =    traffic_data[5];
    double d =    traffic_data[6];

    Vehicle vehicle = Vehicle(x, y, s, d, atan2(vy, vx), magnitude(vx, vy));
    m_vehicles.insert(std::pair<int,Vehicle>(id, vehicle));
  }

}

void TrafficManager::predict()
{
  // contain predictions of all traffic vehicles
  map<int, vector<Vehicle>> predictions;

  // get all traffic vehicles to create prediction of itself into the future


  // return what? prediction? what class holds the prediction data??
}
