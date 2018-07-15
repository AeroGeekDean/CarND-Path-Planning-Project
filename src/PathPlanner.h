/*
 * PathPlanner.h
 *
 *  The PathPlanner class implements the core functionality of this project.
 *  It is associated with an EgoVehicle instance to get its pose data.
 *  It contains the Finite State Machine (FSM) for determining the vehicle behavior.
 *  It generates 'predictive trajectory' for each of these behaviors. Each of these
 *  'predictive trajectory' contains only 2 poses: 1 present and 1 future pose.
 *  From each of these 'predictive trajectories', a cost is assigned based on their
 *  progress on the overall mission. The behavior with the lowest cost is then
 *  chosen as the next FSM state.
 *
 *  Finally, a 'control trajectory' that defines the vehicle's future path is then
 *  generated, to be send to the simulator to control the ego vehicle's motion.
 *  This 'control trajectory' must be smooth enough in order to avoid excessive
 *  acceleration and jerk.
 *
 *  Since the overall mission of the project is to navigate forward through traffic
 *  as fast as possible, while staying under the 50 [mph] speed limit; and each
 *  'predictive trajectories' are already constrained to not violate rules (such
 *  as collision, staying with the 3 permissible lanes, etc)... the cost function
 *  is then simply based on the forward progress only. The behavior (via its
 *  'predictive trajectory') making the most forward progress (for that frame),
 *  is then chosen.
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

  float m_dt_ref;     // [sec]
  float m_time_traj;  // [sec]
  float m_time_probe; // [sec]
  float m_ref_vel;    // [m/s]
  int   m_target_lane;// current maneuver's target lane (could be diff from current lane)
  float m_buffer_distance; // [m] distance to keep back from car ahead
  double m_dv_accel_lim;   // [m/s] delta-velocity limit per time step
  double jerk_lim;    // [m/s^2] delta-accleration limit per time step

 private:

  vector<FSM_state> possibleSuccessorStates();

  vector<Pose> make_predictive_poses(FSM_state state,
                                     const map<int,vector<Pose>>& predictions);

  bool is_vehicle_ahead(int lane,
                        const map<int,vector<Pose>>& predictions,
                        int& car_ahead_id);

  vector<Pose> constant_speed_poses();

  vector<Pose> keep_lane_poses(const map<int,vector<Pose>>& predictions);

  vector<Pose> prep_lane_change_poses(const FSM_state& state,
                                      const map<int,vector<Pose>>& predictions);

  vector<Pose> lane_change_poses(const FSM_state& state,
                                 const map<int,vector<Pose>>& predictions);

  float calculate_cost(Pose pose,
                       const map<int,vector<Pose>>& predictions,
                       const vector<Pose>& trajectory);

  vector<Pose> generate_trajectory(int lane,          // ending lane for trajectory
                                   float v_in,        // speed for trajectory
                                   int num_pp_pts=-1); // -1 = use all previous path points, otherwise specify

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
