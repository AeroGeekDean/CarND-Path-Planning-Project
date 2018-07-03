/*
 * PathPlanner.h
 *
 *  Created on: Jun 29, 2018
 *      Author: deanliu
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include <string>
#include <map>
#include <vector>

class EgoVehicle;
class Pose;

using std::string;
using std::map;
using std::vector;

class PathPlanner {
 public:

  PathPlanner(EgoVehicle& ego);

  virtual ~PathPlanner();

  void updatePrevPath(vector<double> path_x, vector<double> path_y,
                      double end_s, double end_d);

  vector<Pose> chooseNextState(const map<int,vector<Pose>>& predictions );

  vector<Pose> getTrajectoryOutput(int lane,
                                   double vref_in,
                                   int num_pp_pts=-1); // -1 = use all prev path pts, otherwise specify

  float m_dt;  // [sec]
  double m_ref_vel; // [m/s]
  int m_lane;
  double accel_lim; // [m/s] delta-velocity limit per time step
  double jerk_lim; // [m/s^2] delta-accleration limit per time step

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

#endif /* PATHPLANNER_H_ */
