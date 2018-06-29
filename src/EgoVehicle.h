/*
 * EgoVehicle.h
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef EGOVEHICLE_H_
#define EGOVEHICLE_H_

#include "Vehicle.h"
#include <string>
#include <map>

using std::string;
using std::map;

class EgoVehicle : public Vehicle {
 public:
  EgoVehicle(Track& trk);
  EgoVehicle(Track& trk, Pose p);

  virtual ~EgoVehicle();

  vector<Pose> chooseNextState( map<int, vector<Pose>> predictions );

 private:

  vector<string> possibleSuccessorStates();
  vector<Pose> generate_trajectory(string state, map<int, vector<Pose>> predictions);
  float calculate_cost(Pose pose, map<int,vector<Pose>> predictions, vector<Pose> trajectory);

  string m_state;
};

#endif /* EGOVEHICLE_H_ */
