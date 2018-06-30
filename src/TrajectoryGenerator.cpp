/*
 * TrajectoryGenerator.cpp
 *
 *  Created on: Jun 29, 2018
 *      Author: deanliu
 */

#include "TrajectoryGenerator.h"
#include <iterator>
#include "EgoVehicle.h"
#include "UtilFunctions.h"
#include "spline.h"

TrajectoryGenerator::TrajectoryGenerator(EgoVehicle& ego)
:m_rEgo(ego)
{
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}

void TrajectoryGenerator::updatePrevPath(vector<double> path_x, vector<double> path_y,
                                         double end_s, double end_d)
{
  m_prev_path_x = path_x;
  m_prev_path_y = path_y;
  m_end_path_s = end_s;
  m_end_path_d = end_d;
  m_prev_path_size = path_x.size();
}

vector<vector<double>> TrajectoryGenerator::getTrajectoryOutput()
{
  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m TODO: HANGE THIS
  // Later we will interpolate these waypoints with a spline and fill it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x,y, yaw states
  // we will reference the starting point as either where the car is, or at the previous path end point
  double ref_x;
  double ref_y;
  double ref_yaw;

  // fill starting point with 2 points, to establish spline starting angle
  if (m_prev_path_size < 2)  // if previous size is almost empty use the car as starting reference
  {
    ref_x   = m_rEgo.getPose().x;
    ref_y   = m_rEgo.getPose().y;
    ref_yaw = m_rEgo.getPose().yaw;

    // find another point behind the car, to make the spline tangent to the car
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
  }
  else  // use the previous path's end point(s) as starting reference
  {
    ref_x = m_prev_path_x[m_prev_path_size-1];
    ref_y = m_prev_path_y[m_prev_path_size-1];

    double ref_x_prev = m_prev_path_x[m_prev_path_size-2];
    double ref_y_prev = m_prev_path_y[m_prev_path_size-2];

    ref_yaw = atan2( (ref_y-ref_y_prev), (ref_x-ref_x_prev) );

    // Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly 30m spaced points ahead of the starting reference
  double d = 2 + 4*m_lane;

  vector<double> next_wp0 = m_rEgo.m_rTrack.getXY(m_rEgo.getPose().s +30, d);
  vector<double> next_wp1 = m_rEgo.m_rTrack.getXY(m_rEgo.getPose().s +60, d);
  vector<double> next_wp2 = m_rEgo.m_rTrack.getXY(m_rEgo.getPose().s +90, d);

  ptsx.push_back( next_wp0[0] );
  ptsx.push_back( next_wp1[0] );
  ptsx.push_back( next_wp2[0] );

  ptsy.push_back( next_wp0[1] );
  ptsy.push_back( next_wp1[1] );
  ptsy.push_back( next_wp2[1] );

  // transform from global to local coordinate (of ref_x/y/yaw location)
  for (int i=0; i<ptsx.size(); i++)
  {
    vector<double> out = global2local(ptsx[i], ptsy[i], ref_x, ref_y, (0-ref_yaw));
    ptsx[i] = out[0];
    ptsy[i] = out[1];
  }

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // Define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Start with all of the previous path points from the last frame
  for (int i=0; i<m_prev_path_size; i++)
  { // these are in global (x,y) coord
    next_x_vals.push_back(m_prev_path_x[i]);
    next_y_vals.push_back(m_prev_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = magnitude(target_x, target_y);

  double dx = m_dt*m_ref_vel * (target_x/target_dist); // s_distance per time step
  double x_point = dx;

  // Fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 points. Note: need to convert from local back to global coord.
  for (int i=0; i<(50-m_prev_path_size); i++)
  {
    vector<double> out = local2global(x_point, s(x_point), ref_x, ref_y, (ref_yaw));
    next_x_vals.push_back(out[0]);
    next_y_vals.push_back(out[1]);
    x_point += dx;
  }

  return { next_x_vals, next_y_vals };
}

vector<Pose> TrajectoryGenerator::chooseNextState(const map<int,vector<Pose>>& predictions)
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

  // Find the return the lowest cost trajectory
  vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
  int best_idx = std::distance(begin(costs), best_cost);
  return trajectories[best_idx];
}


// TODO:: This is a temp fake stub. Implement real one by designing in cost.h/cost.cpp from class room example
float TrajectoryGenerator::calculate_cost(Pose pose,
                                          const map<int,vector<Pose>>& predictions,
                                          const vector<Pose>& trajectory)
{
  float cost = 0;
  // TODO
  return cost;
}

vector<string> TrajectoryGenerator::possibleSuccessorStates()
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

vector<Pose> TrajectoryGenerator::generate_trajectory(string state,
                                                      const map<int,vector<Pose>>& predictions)
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
//      trajectory = keep_lane_trajectory(predictions);
  }
  else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
  {
//      trajectory = lane_change_trajectory(state, predictions);
  }
  else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
  {
//      trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

vector<Pose> TrajectoryGenerator::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
//    float next_pos = position_at(1);
    vector<Pose> trajectory;// = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
                               //   Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}
