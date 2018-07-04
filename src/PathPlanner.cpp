/*
 * PathPlanner.cpp
 *
 *  Created on: Jun 29, 2018
 *      Author: deanliu
 */

#include "PathPlanner.h"

#include <iostream>
#include "EgoVehicle.h"
#include "PathPlanner.h"
#include "UtilFunctions.h"
#include "spline.h"

using std::cout;
using std::endl;
using std::min;
using std::max;

PathPlanner::PathPlanner(EgoVehicle& ego)
:m_rEgo(ego)
{
}

PathPlanner::~PathPlanner()
{
}

/*------------------
 * updatePrevPath()
  ------------------*/
void PathPlanner::updatePrevPath(vector<double> path_x,
                                 vector<double> path_y,
                                 double end_s,
                                 double end_d) {
  m_prev_path_x = path_x;
  m_prev_path_y = path_y;
  m_prev_path_size = path_x.size();

  if (m_prev_path_size==0) {
    // these are undefined/zero when there's is no prev path (simulator 1st frame)
    m_end_path_s = m_rEgo.getPose().s;
    m_end_path_d = m_rEgo.getPose().d;
  }else{
    m_end_path_s = end_s;
    m_end_path_d = end_d;
  }
}

/*-----------------------
 * getTrajectoryOutput()
  -----------------------*/
vector<Pose> PathPlanner::getTrajectoryOutput(int lane, float vref_in, float dt_in, float traj_time, int num_pp_pts)
{
  /*
   * Create a set of widely spaced (x,y) 5 waypoints. The first 2 pts defines the starting
   * reference location and orientation, then 3 more evenly-spaced points far ahead of the vehicle.
   * A spline is then fitted to these 5 points, and finer spaced points are interpolated for the
   * vehicle trajectory. The spacing of these trajectory points will define the vehicle speed
   * (and acceleration).
   */

  int num_steps = (int)ceil(traj_time/dt_in); // number of points to build the trajectory with

  vector<double> ptsx;
  vector<double> ptsy;

  /*
   * Here we allow caller to specify how many points from the previous path to re-use.
   * If num_pp_pts = -1, then use ALL the points. We also check to make there is sufficient points to re-use
   */
  int ref_ppath_size;
  if ((num_pp_pts == -1) || (m_prev_path_size < num_pp_pts))
    ref_ppath_size = m_prev_path_size;
  else
    ref_ppath_size = num_pp_pts;

  // --- Determine reference x,y, yaw ---
  // either car current state, or end of previous path points

  double ref_x;
  double ref_y;
  double ref_s;
  double ref_yaw;
  double ref_spd;

  if (ref_ppath_size < 3)  // if not sufficient points, use the car as starting reference
  {
    ref_x   = m_rEgo.getPose().x;
    ref_y   = m_rEgo.getPose().y;
    ref_s   = m_rEgo.getPose().s;
    ref_yaw = m_rEgo.getPose().yaw;
    ref_spd = m_rEgo.getPose().spd;

    // find another point slight behind the car, to force the spline tangent to the car
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
  }
  else  // use prev path as starting reference
  {
    ref_x = m_prev_path_x[ref_ppath_size-1];
    ref_y = m_prev_path_y[ref_ppath_size-1];

    double ref_x_prev = m_prev_path_x[ref_ppath_size-2];
    double ref_y_prev = m_prev_path_y[ref_ppath_size-2];

    ref_yaw   = atan2( (ref_y-ref_y_prev), (ref_x-ref_x_prev) );
    double dist = distance(ref_x, ref_y, ref_x_prev, ref_y_prev);
    ref_spd   = dist/m_dt_ref;  // previous path was built using dt_ref

    vector<double> tmp = m_rEgo.m_rTrack.getFrenet(ref_x, ref_y, ref_yaw);
    ref_s = tmp[0];

    // Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // --- Determine Frenet D (for ptsx/y) ---

  double ref_d = 2 + 4*lane;

  // In Frenet add evenly 30m spaced points ahead of the reference
  vector<double> next_wp0 = m_rEgo.m_rTrack.getXY(ref_s+30, ref_d);
  ptsx.push_back( next_wp0.at(0) );
  ptsy.push_back( next_wp0.at(1) );

  vector<double> next_wp1 = m_rEgo.m_rTrack.getXY(ref_s+60, ref_d);
  ptsx.push_back( next_wp1.at(0) );
  ptsy.push_back( next_wp1.at(1) );

  vector<double> next_wp2 = m_rEgo.m_rTrack.getXY(ref_s+90, ref_d);
  ptsx.push_back( next_wp2.at(0) );
  ptsy.push_back( next_wp2.at(1) );

  // TRANSFORM from global to local coordinate (of ref_x/y/yaw location)
  for (int i=0; i<ptsx.size(); i++)
  {
    vector<double> out = global2local(ptsx.at(i), ptsy.at(i), ref_x, ref_y, (0-ref_yaw));
    ptsx.at(i) = out.at(0);
    ptsy.at(i) = out.at(1);
  }

  // create and set (x,y) points to a spline
  tk::spline s;
  s.set_points(ptsx, ptsy);

  // output container
  vector<Pose>   trajectory;

  // Start with portion of the previous path points from the last frame
  for (int i=0; i<ref_ppath_size; i++)
  { // these are in global (x,y) coord
    Pose p;
      p.x = m_prev_path_x[i];
      p.y = m_prev_path_y[i];
    trajectory.push_back(p);
  }

  /*
   * Since our given speed is in S direction, we need to do a little bit of trigonometry
   * to find our step size in the local X direction. They are aligned right at the vehicle,
   * but will diverge as the trajectory curves. We'll use the previous step slope to predict.
   * each future step slope. We'll also use Trig Identity equation to calc `cosine_ratio` below,
   * so as to avoid trig functions which are computationally costly.
   */

//  double v_prev = ref_spd;
//  double a_prev = ref_accel;
//  double a_tgt_dt, j_tgt_dt; // accel & jerk target, per frame
  double ds, x, y, slope;
  double cosine_ratio = 1.0; // cosine of slope angle, angle = atan(slope).
  double x_prev = 0.0;
  double y_prev = 0.0;

  int steps = max( (num_steps-ref_ppath_size), 0);
  for (int i=0; i<steps; i++)
  {
    /* --- SPEED CONTROLLER (currently not working) ---
     * Compute allowable speed for this step, subject to accel & jerk limits
     * This means vref_in could have discontinuous big jumps, since the actual speed is protected.
     */
//    a_tgt_dt = min(max((vref_in - v_prev), -accel_lim), accel_lim);
//    j_tgt_dt = min(max((a_tgt_dt - a_prev*dt_in), -jerk_lim), jerk_lim);
//    cout << a_tgt_dt << ", ";
//    a_prev += j_tgt_dt;
//    v_prev += a_prev * dt_in;
//    v_prev = min(v_prev, 49.5*mph2ms);

    ds = vref_in * dt_in;
    x += cosine_ratio*ds;  // to find dx based on yaw
    y = s(x);

    vector<double> out = local2global(x, y, ref_x, ref_y, (ref_yaw));
    Pose p;
      p.x = out[0];
      p.y = out[1];
    trajectory.push_back(p);

//    cosine_ratio = cos( atan2((y_point-y_point_prev), (x_point-x_point_prev)) );
    slope = (y-y_prev)/(x-x_prev);
    cosine_ratio = 1/sqrt(1+slope*slope);  // find new value
    x_prev = x;
    y_prev = y;
  }

  return trajectory;
}


/*-------------------
 * chooseNextState()
  -------------------*/
vector<Pose> PathPlanner::chooseNextState(const map<int,vector<Pose>>& predictions)
{
  /*

  ***Here you can implement the transition_function code from the Behavior Planning Pseudocode
  classroom concept.***

  INPUT: A predictions map. This is a map using vehicle id as keys with predicted
      vehicle trajectories as values. A trajectory is a vector of Vehicle::Pose objects. The first
      item in the trajectory represents the vehicle at the current time step. The second item in
      the trajectory represents the vehicle one time step in the future, etc.
  OUTPUT: The the best (lowest cost) trajectory for the ego vehicle corresponding to the next
      ego vehicle state.

  Functions that will be useful:
  1. successor_states() - Uses the current state to return a vector of possible successor states
     for the finite state machine.
  2. generate_trajectory(string state, map<int, vector<Vehicle>> predictions) - Returns a vector
     of Vehicle objects representing a vehicle trajectory, given a state and predictions. Note
     that trajectory vectors might have size 0 if no possible trajectory exists for the state.
  3. calculate_cost(Vehicle vehicle, map<int, vector<Vehicle>> predictions, vector<Vehicle> trajectory)
     - Included from cost.cpp, computes the cost for a trajectory.
  */

  // Generate trajectory associated with each state, then calculate their associated cost.

  vector<string> states = possibleSuccessorStates();

  vector<float> costs;

  vector<vector<Pose>> trajectories;

  for (auto state_itr=states.begin(); state_itr!=states.end(); state_itr++)
  {
    vector<Pose> trajectory = generate_trajectory(*state_itr, predictions);
    if (trajectory.size() != 0)
    {
      float cost = calculate_cost(m_rEgo.getPose(), predictions, trajectory);
      costs.push_back(cost);
      trajectories.push_back(trajectory);
    }
  }

  // Find and return the lowest cost trajectory
  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = std::distance(begin(costs), best_cost);
  return trajectories.at(best_idx);

  // build the actual trajectory, using dt & look ahead time for control (instead of probing)
//  return getTrajectoryOutput(m_target_lane, )
}

/*------------------
 * calculate_cost()
  ------------------*/
// TODO:: This is a temp fake stub. Implement real one by designing in cost.h/cost.cpp from class room example
float PathPlanner::calculate_cost(Pose pose,
                                  const map<int,vector<Pose>>& predictions,
                                  const vector<Pose>& trajectory)
{
  float cost = 0;
  // TODO
  return cost;
}

/*---------------------------
 * possibleSuccessorStates()
  ---------------------------*/
vector<string> PathPlanner::possibleSuccessorStates()
{
  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  vector<string> states;
  states.push_back("KL"); // KL is always a viable state

  string state = m_state;
  if(state.compare("KL") == 0)
  {
      states.push_back("PLCL");
      states.push_back("PLCR");
  }
  else if (state.compare("PLCL") == 0)
  {
      if (m_rEgo.getLane() != 0) // not in far left lane
      {
          states.push_back("PLCL");
          states.push_back("LCL");
      }
  }
  else if (state.compare("PLCR") == 0)
  {
      if (m_rEgo.getLane() != m_rEgo.m_rTrack.m_num_lanes_available-1) // not in far right lane
      {
          states.push_back("PLCR");
          states.push_back("LCR");
      }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

/*-----------------------
 * generate_trajectory()
  -----------------------*/
vector<Pose> PathPlanner::generate_trajectory(string state, const map<int,vector<Pose>>& predictions)
{
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */

  vector<Pose> trajectory;

  if (state.compare("CS") == 0)
  {
      trajectory = constant_speed_trajectory();
  }
  else if (state.compare("KL") == 0)
  {
      trajectory = keep_lane_trajectory(predictions);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
  {
      trajectory = lane_change_trajectory(state, predictions);
  }
  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
  {
//      trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

/*-----------------------------
 * constant_speed_trajectory()
  -----------------------------*/
vector<Pose> PathPlanner::constant_speed_trajectory()
{
  // Continue on ego's current lane and speed
  return {m_rEgo.getPose(), m_rEgo.propagatePose(m_rEgo.getPose(),m_time_probe)};
}

/*------------------------
 * keep_lane_trajectory()
  ------------------------*/
vector<Pose> PathPlanner::keep_lane_trajectory(const map<int,vector<Pose>>& predictions)
{
  // Continue on ego's current lane
  // But if there're traffic ahead, adjust speed to 'match' with them.

  // Note: need to save this data somehow, so that if this state is chosen,
  //       the data could be re-used when the actual trajectory is build

  float max_s = m_rEgo.m_rTrack.m_max_s;
  float closest_dist_ahead = max_s/2;
  int car_ahead_id = -1;

  for (auto itr=predictions.begin(); itr!=predictions.end(); itr++)
  {
    const Pose& traffic = itr->second.at(0);
    if (traffic.lane == m_target_lane) // filter by lane
    {
      float traffic_s = traffic.s;
      float dist_ahead = traffic.s - m_rEgo.getPose().s;
        // protect around start/finish line when s wraps back to zero
        while (dist_ahead >   (max_s/2)) dist_ahead -= max_s;
        while (dist_ahead <= -(max_s/2)) dist_ahead += max_s;

      // look for the car directly ahead of us
      if ((dist_ahead > 0) && (dist_ahead < closest_dist_ahead))
      {
        car_ahead_id = itr->first;
        closest_dist_ahead = dist_ahead;
      }
    }
  }


  if (car_ahead_id == -1) // no cars ahead of us, keep on trucking on!
  {
    return constant_speed_trajectory();
  }
  else
  {
    // let's evaluate predictions in the future...
    const Pose& car_ahead = predictions.at(car_ahead_id).back();
    Pose ego_now = m_rEgo.getPose();
    Pose ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);

    if ( (car_ahead.s-ego_future.s) < 30) // closer than 30m, let's slow to match speed
    {
      ego_now.s = car_ahead.spd;
      return {m_rEgo.getPose(), m_rEgo.propagatePose(ego_now, m_time_probe)};
    }
    else // car ahead is still far away, keep on trucking on!
    {
      return constant_speed_trajectory();
    }
  }

}

/*--------------------------
 * lane_change_trajectory()
  --------------------------*/
vector<Pose> PathPlanner::lane_change_trajectory(const string& state, const map<int,vector<Pose>>& predictions)
{
  vector<Pose> trajectory;
  return trajectory;
}

/*-------------------------------
 * prep_lane_change_trajectory()
  -------------------------------*/
vector<Pose> PathPlanner::prep_lane_change_trajectory(const string& state, const map<int,vector<Pose>>& predictions)
{
  vector<Pose> trajectory;
  return trajectory;
}



