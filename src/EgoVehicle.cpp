/*
 * EgoVehicle.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#include "EgoVehicle.h"
#include <iterator>

EgoVehicle::EgoVehicle(Track& trk)
:Vehicle(trk)
{

}

EgoVehicle::EgoVehicle(Track& trk, Pose p)
:Vehicle(trk, p)
{

}

EgoVehicle::~EgoVehicle()
{

}

vector<Vehicle::Pose> EgoVehicle::chooseNextState(map<int, vector<Pose>> predictions)
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
      float cost = calculate_cost(m_pose, predictions, trajectory);
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
float EgoVehicle::calculate_cost(Pose pose, map<int,vector<Pose>> predictions, vector<Pose> trajectory)
{
  float cost = 0;
  // TODO
  return cost;
}

vector<string> EgoVehicle::possibleSuccessorStates()
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
      if (lane != 0) // far left lane
      {
          states.push_back("PLCL");
          states.push_back("LCL");
      }
  }
  else if (state.compare("PLCR") == 0)
  {
      if (lane != lanes_available-1) // far right lane
      {
          states.push_back("PLCR");
          states.push_back("LCR");
      }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle::Pose> EgoVehicle::generate_trajectory(string state,
                                                      map<int,vector<Pose>> predictions)
{
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */
  vector<Vehicle::Pose> trajectory;

  if (state.compare("CS") == 0)
  {
//      trajectory = constant_speed_trajectory();
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

