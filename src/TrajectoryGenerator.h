/*
 * TrajectoryGenerator.h
 *
 *  Created on: Jun 29, 2018
 *      Author: deanliu
 */

#ifndef TRAJECTORYGENERATOR_H_
#define TRAJECTORYGENERATOR_H_

#include <string>
#include <map>
#include <vector>

class EgoVehicle;
class Pose;

using std::string;
using std::map;
using std::vector;

class TrajectoryGenerator {
 public:

  TrajectoryGenerator(EgoVehicle& ego);

  virtual ~TrajectoryGenerator();

  void updatePrevPath(vector<double> path_x, vector<double> path_y,
                      double end_s, double end_d);

  vector<Pose> chooseNextState(const map<int,vector<Pose>>& predictions );

  vector<vector<double>> getTrajectoryOutput();

  float m_dt;  // [sec]
  double m_ref_vel; // [m/s]
  int m_lane;

 private:

  vector<string> possibleSuccessorStates();

  vector<Pose> generate_trajectory(string state,
                                   const map<int,vector<Pose>>& predictions);

  vector<Pose> constant_speed_trajectory();

  float calculate_cost(Pose pose,
                       const map<int,vector<Pose>>& predictions,
                       const vector<Pose>& trajectory);

  string m_state;

  float m_time_horizon;  // [sec]

  // info on previous path, returned from simulator
  vector<double> m_prev_path_x;
  vector<double> m_prev_path_y;
  double         m_end_path_s;
  double         m_end_path_d;
  int            m_prev_path_size;

  EgoVehicle& m_rEgo;
};

#endif /* TRAJECTORYGENERATOR_H_ */
