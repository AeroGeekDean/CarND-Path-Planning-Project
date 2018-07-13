/*
 * PathPlanner.h
 *
 *  Created on: Jun 29, 2018
 *      Author: deanliu
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "Pose.h"
#include <string>
#include <map>
#include <vector>

class EgoVehicle;

using std::string;
using std::map;
using std::vector;

class PathPlanner {
 public:

  PathPlanner(EgoVehicle& ego);

  virtual ~PathPlanner();

  void updatePrevPath(const vector<double>& path_x,
                      const vector<double>& path_y,
                      double end_s, double end_d);

  vector<Pose> chooseNextState(const map<int,vector<Pose>>& predictions );

  vector<Pose> getTrajectoryOutput(int lane,          // ending lane for trajectory
                                   float v_in,        // speed for trajectory
                                   float dt_in,       // dt, coarser for probing, finer for actual trajectory
                                   float traj_time,   // look ahead time the trajectory is build out to
                                   int num_pp_pts=-1); // -1 = use all prev path pts, otherwise specify

  float m_dt_ref;     // [sec]
  float m_time_traj;  // [sec]
  float m_time_probe; // [sec]
  float m_ref_vel;    // [m/s]
  int   m_target_lane;// current maneuver's target lane (could be diff from current lane when changing lane)
  float m_buffer_distance; // [m] distance to keep back from car ahead
  double m_dv_accel_lim;   // [m/s] delta-velocity limit per time step
  double jerk_lim;    // [m/s^2] delta-accleration limit per time step

 private:

  vector<FSM_state> possibleSuccessorStates();

  vector<Pose> generate_trajectory(FSM_state state,
                                   const map<int,vector<Pose>>& predictions);

//  int get_id_vehicle_ahead(int lane, const map<int,vector<Pose>>& predictions);
  bool is_vehicle_ahead(int lane,
                        const map<int,vector<Pose>>& predictions,
                        int& car_ahead_id);

  vector<Pose> constant_speed_trajectory();
  vector<Pose> keep_lane_trajectory(const map<int,vector<Pose>>& predictions);
  vector<Pose> prep_lane_change_trajectory(const FSM_state& state,
                                           const map<int,vector<Pose>>& predictions);
  vector<Pose> lane_change_trajectory(const FSM_state& state,
                                      const map<int,vector<Pose>>& predictions);

  float calculate_cost(Pose pose,
                       const map<int,vector<Pose>>& predictions,
                       const vector<Pose>& trajectory);

  FSM_state m_state = kFSM_Start;

  // info on previous path, returned from simulator
  // used to build the control trajectory
  vector<double> m_prev_path_x;
  vector<double> m_prev_path_y;
  double         m_end_path_s;
  double         m_end_path_d;
  int            m_prev_path_size;

  EgoVehicle& m_rEgo;
};

#endif /* PATHPLANNER_H_ */
