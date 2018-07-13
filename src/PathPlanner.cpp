/*
 * PathPlanner.cpp
 *
 *  Created on: Jun 29, 2018
 *      Author: deanliu
 */

#include "PathPlanner.h"
#include "EgoVehicle.h"
#include "UtilFunctions.h"
#include "spline.h"
#include <iostream>
#include <iomanip>

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
void PathPlanner::updatePrevPath(const vector<double>& path_x,
                                 const vector<double>& path_y,
                                 double end_s,
                                 double end_d) {
  m_prev_path_x     = path_x;
  m_prev_path_y     = path_y;
  m_prev_path_size  = path_x.size();

  if (m_prev_path_size==0)
  { // these are undefined/zero when there's is no prev path (simulator 1st frame)
    m_end_path_s = m_rEgo.getPose().s;
    m_end_path_d = m_rEgo.getPose().d;
  }
  else
  {
    m_end_path_s = end_s;
    m_end_path_d = end_d;
  }
}

/*-----------------------
 * getTrajectoryOutput()
  -----------------------*/
vector<Pose> PathPlanner::getTrajectoryOutput(int lane, float v_in, float dt_in, float traj_time, int num_pp_pts)
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
   * If num_pp_pts = -1, then use ALL the points. We also check to make there is
   * sufficient points to re-use.
   */
  int ref_ppath_size; // reference previous path size
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
//  double ref_accel;

  if (ref_ppath_size < 3)  // if not sufficient points, use the car as starting reference
  {
    ref_x   = m_rEgo.getPose().x;
    ref_y   = m_rEgo.getPose().y;
    ref_s   = m_rEgo.getPose().s;
    ref_yaw = m_rEgo.getPose().yaw;
    ref_spd = m_rEgo.getPose().spd;
//    ref_accel = 0.0;

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
    ref_spd   = dist/dt_in;  // previous path was built using dt_ref

//    double ref_x_pprev = m_prev_path_x[ref_ppath_size-3];
//    double ref_y_pprev = m_prev_path_y[ref_ppath_size-3];
//    double dist2 = distance(ref_x_prev, ref_y_prev, ref_x_pprev, ref_y_pprev);
//    ref_accel = (ref_spd - dist2/dt_in)/dt_in;

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
  tk::spline s_curve;
  s_curve.set_points(ptsx, ptsy);

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


  /* --- SPEED CONTROLLER ---
   *
   *  I tried to implement a 2nd order lag filter, like in the block diagram below.
   *  And choosing the gains (gain_a, gain_j) judicially using engineering judgment.
   *
   *          V_err         A_cmd   A_err          Jerk      Accel
   * Vin --->o----->[gain_a]----->o------>[gain_j]---->[1/S]--+--->[1/S]--+--> Vout
   *     (+) ^                (+) ^                           |           |
   *         |(-)                 |(-)   Accel_feedback       |           |
   *         |                    +---------------------------+           |
   *         |    Velocity_feedback                                       |
   *         +------------------------------------------------------------+
   *
   *  The associated 2nd order transfer function (Laplace transform):
   *
   *    Vout/Vin = (gain_a*gain_j) / ( S^2 + gain_j*S + gain_a*gain_j )
   *
   *  where:
   *    Omega_n (natural frequency) = sqrt( gain_a*gain_j )
   *    zeta (damping ratio) = 0.5 * gain_j / Omega_n
   *
   *  Thus with:
   *    gain_a = 0.403 [1/sec],       gain_j = 0.9 [1/sec]     (set above...)
   *
   *  The filter dynamics are:
   *    Omega_n = 0.602 [rad/sec],    zeta = 0.747 <--- under-damped, oscillatory!!!
   *
   *  However, I was having trouble debugging out the long-period speed oscillation and ending
   *  up spending too much time on controller design, instead of project path planning design.
   *
   *  Thus abandoning this approach, for now.
   *
   *  I instead IMPLEMENT A MUCH SIMPLER "LIMITED ACCELERATION SPEED CONTROLLER" below.
   *
   *  By limiting the max acceleration to 5.0 [m/s^2], and implemented it as a
   *  max-speed-change-per-frame (m_dv_accel_lim) of 5.0*dt = 5.0*0.02 = 0.1 [m/s].
   *  This yields a max jerk of only 2.5 [m/s^3] for a large step speed input...
   *  ONLY when a large step speed input is immediately followed by a large step in opposite direction,
   *  does the max jerk reach 10.0 [m/s^3].
   *
   *  ---- below are variables associated with the orig design.
   *          Preserving, so I can come back to them later. ---
   */

//  double a_prev = ref_accel;  // acceleration at the end of previous path points
  double v_prev = ref_spd;      // velocity at the end of previous path points

  /* SPEED CONTROLLER gain_a (below) is chosen this way. Given that:
   * - Estimated max step delta velocity command is 50 [mph] = 22.35 [m/s],
   * - accel_lim = 10 m/s^2 -> allow 90% of that
   * - hence: 9/22.35 = 0.403
   */
//  float gain_a = 0.403; // accel_cmd per velocity_error, [1/sec],

    /* Similarly, SPEED CONTROLLER gain_j (below) is chosen by... Given that:
     * - max accel = 10.0 [m/s^2],
     * - jerk_lim = 10 m/s^3 -> allow 90% of that
     * - hence: 9/10 = 0.9
     */
//  float gain_j = 0.9;   // jerk_cmd per acceleration_error, [1/sec], j_lim = 10 m/s^3 -> use 9.0, 9/10 = 0.9
//  double j;

  double dv;  // delta_speed per frame
  double ds, y, slope;
  double x = 0.0;
  double x_prev = 0.0; // origin of body axis
  double y_prev = 0.0;

  /* CONVERSION OF SPEED & X_STEP, FROM FRENET-S DIRECTION TO BODY-X DIRECTION....
   * Since our given speed is in S direction, we need to do a little bit of trigonometry
   * to find our step size in the local X direction. S & X are co-aligned right near the vehicle,
   * but will diverge as the trajectory forward curves & bends. We'll use the previous step 'slope'
   * to predict each future step 'slope'. Furthermore, We'll also use Trigonometry Identity equation
   * to calculate `cosine_ratio` below, so as to avoid using the computationally costly trigonometry
   * functions.
   */
  double cosine_ratio = 1.0; // cosine of slope angle, angle = atan(slope) = atan(dy/dx)

  int steps = max( (num_steps-ref_ppath_size), 0); // remaining # of trajectory points to build

  for (int i=0; i<steps; i++)
  {
//--- These are the 2nd order lag filter speed controller code ---
//    Saving them here so I can come back to them in the future.
//
//    j = gain_j*(gain_a*(vref_in - v_prev) - a_prev);
//    a_prev += j*dt_in;
//    v_prev += a_prev*dt_in;
//    v_prev = min((v_prev + a_prev*dt_in), (double)m_ref_vel);

    // rate limit how much delta_v is permitted per frame (hence acceleration limiting)
    dv = max(min((v_in - v_prev), m_dv_accel_lim), -m_dv_accel_lim);
    v_prev += dv;

    ds = v_prev * dt_in;
    x += cosine_ratio*ds;  // to find dx based on yaw
    y = s_curve(x);

    vector<double> out = local2global(x, y, ref_x, ref_y, ref_yaw);
    Pose p;
      p.x = out[0];
      p.y = out[1];
    trajectory.push_back(p);

/*  Actual cosine_ratio = cos( atan2((y_point-y_point_prev), (x_point-x_point_prev)) );
 *  Below is simpler formula using trig identities
 */
    slope = (y-y_prev)/(x-x_prev);
    cosine_ratio = 1/sqrt(1+slope*slope);  // find new value

    // save previous values
    x_prev = x;
    y_prev = y;
  }

//  cout << "TrajCalc_v: [";
//  double xp = 0, yp = 0;
//  for (auto p : trajectory) {
//    cout << distance(p.x, p.y, xp, yp)/m_dt_ref << ", ";
//    xp=p.x; yp=p.y;
//  }
//  cout << "]" << endl;

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

  vector<FSM_state> states = possibleSuccessorStates();

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

  // extract final state of best trajectory
  const Pose& p = trajectories.at(best_idx).back();
  m_state = p.state; // update PathPlanner FSM's state
  m_target_lane = p.lane;

  cout << "Spd: " << p.spd << ", S: " << p.s << ", State: " << m_state << "[" << m_target_lane << "]: ";
  for (int i=0; i<costs.size(); i++) {
    if (i==best_idx)
      cout << trajectories.at(i).back().state << "(" << (costs.at(i)-*best_cost) << "), ";
    else
      cout << trajectories.at(i).back().state << " " << (costs.at(i)-*best_cost) << " , ";
  } cout << endl;
  cout<<endl;

  // build the actual trajectory, using dt & look ahead time for control (instead of probing)
  return getTrajectoryOutput(p.lane, p.spd, m_dt_ref, m_time_traj, 10);
}

/*------------------
 * calculate_cost()
  ------------------*/
// TODO:: This is a temp fake stub. Implement real one by designing in cost.h/cost.cpp from class room example
float PathPlanner::calculate_cost(Pose pose,
                                  const map<int,vector<Pose>>& predictions,
                                  const vector<Pose>& trajectory)
{
  float cost = 1;
  float dist_fwd = s_wrap(trajectory.back().s - pose.s);
  cost = -dist_fwd;
  return cost;
}

/*---------------------------
 * possibleSuccessorStates()
  ---------------------------*/
vector<FSM_state> PathPlanner::possibleSuccessorStates()
{
  /*
  Provides the possible next states given the current state for the FSM
  discussed in the course, with the exception that lane changes happen
  instantaneously, so LCL and LCR can only transition back to KL.
  */
  vector<FSM_state> states;
  states.push_back(kFSM_KL); // KL is always a viable state

  FSM_state state = m_state;
//  string state = m_rEgo.getPose().state;
  switch (m_state)
  {
    case kFSM_KL:
      if (m_target_lane != 0) // not in far left lane
        states.push_back(kFSM_PLCL);
      if (m_target_lane != m_rEgo.m_rTrack.m_num_lanes_available-1) // not in far right lane
        states.push_back(kFSM_PLCR);
      break;

    case kFSM_PLCL:
      if (m_target_lane != 0) // not in far left lane
      {
        states.push_back(kFSM_PLCL);
        states.push_back(kFSM_LCL);
      }
      break;

    case kFSM_PLCR:
      if (m_target_lane != m_rEgo.m_rTrack.m_num_lanes_available-1) // not in far right lane
      {
        states.push_back(kFSM_PLCR);
        states.push_back(kFSM_LCR);
      }
      break;

    default:
      //If state is "LCL" or "LCR", then just return "KL"
      break;
  }

  return states;
}

/*-----------------------
 * generate_trajectory()
  -----------------------*/
vector<Pose> PathPlanner::generate_trajectory(FSM_state state, const map<int,vector<Pose>>& predictions)
{
  /*
  Given a possible next state, generate the appropriate trajectory to realize the next state.
  */

  vector<Pose> trajectory;

  switch (state)
  {
    case kFSM_CS:
      trajectory = constant_speed_trajectory();  // for traffic vehicle only
      break;

    case kFSM_KL:
      trajectory = keep_lane_trajectory(predictions);
      break;

    case kFSM_LCL:
    case kFSM_LCR:
      trajectory = lane_change_trajectory(state, predictions);
      break;

    case kFSM_PLCL:
    case kFSM_PLCR:
      trajectory = prep_lane_change_trajectory(state, predictions);
      break;

    default:
      break;
  }

  return trajectory;
}

/*-----------------------------
 * constant_speed_trajectory()
  -----------------------------*/
vector<Pose> PathPlanner::constant_speed_trajectory()
{
  // Continue on ego's current lane and speed
  Pose ego_now = m_rEgo.getPose();
  ego_now.state = kFSM_CS;

  return {m_rEgo.getPose(), m_rEgo.propagatePose(ego_now, m_time_probe)};
}

/*------------------------
 * keep_lane_trajectory()
  ------------------------*/
vector<Pose> PathPlanner::keep_lane_trajectory(const map<int,vector<Pose>>& predictions)
{
  // Continue on ego's current lane
  // But if there're traffic ahead, adjust speed to 'match' with them.
  // 'Match' == same speed but some distance behind the lead vehicle

  // Note: need to save this data somehow(?), so that if this state is chosen,
  //       the data could be re-used when the actual trajectory is build

  int id_car_ahead;

  Pose ego_now = m_rEgo.getPose();
  ego_now.state = kFSM_KL;
  ego_now.lane = m_target_lane; // follow last target lane, as ego might be in maneuver transient

  Pose ego_future;

  // First, find the car that is ahead of us, if any
  if (is_vehicle_ahead(m_target_lane, predictions, id_car_ahead))
  {
    // let's evaluate predictions in the future...
    const Pose& car_ahead = predictions.at(id_car_ahead).back(); // grab future pose
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);

    if ( s_wrap2(car_ahead.s-ego_future.s) < m_buffer_distance) // closer than buffer, let's slow to match speed
    {
      // back calc spd needed to reach desired s...
      float ego_future_desired_s = s_wrap(car_ahead.s-m_buffer_distance);
      Pose ego_tmp = ego_now;
      ego_tmp.spd = min(s_wrap2(ego_future_desired_s-ego_tmp.s)/m_time_probe, ego_tmp.spd);
      ego_future = m_rEgo.propagatePose(ego_tmp, m_time_probe);

      ego_future.spd = car_ahead.spd;
//cout << "KL - car ahead too close! New speed = " << ego_future.spd << endl;
    }
    else // car ahead is still far away, maintain speed and keep on trucking on!
    {
//cout << "KL - car ahead still far!" << endl;
      ego_now.spd = m_ref_vel;
      ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
    }
  }
  else
  { // no cars ahead, keep on going at ref_vel !
    ego_now.spd = m_ref_vel;
//cout << "KL - lane empty!" << endl;
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
  }

  return {ego_now, ego_future};

}


/*-------------------------------
 * prep_lane_change_trajectory()
  -------------------------------*/
vector<Pose> PathPlanner::prep_lane_change_trajectory(const FSM_state& state, const map<int,vector<Pose>>& predictions)
{
  int new_lane = m_target_lane;
  if (state==kFSM_PLCR)
    new_lane += 1;
  else if (state==kFSM_PLCL)
    new_lane -= 1;

  int id_car_ahead;

  const vector<Pose>& kl_data = keep_lane_trajectory(predictions);
  const Pose& ego_now_kl = kl_data.front();
  const Pose& ego_future_kl = kl_data.back();

  Pose ego_now = m_rEgo.getPose();
  ego_now.state = state;
  ego_now.lane = m_target_lane; // follow last target lane, as ego might be in maneuver transient

  Pose ego_future;

  // First, find the car that is ahead of us in new lane, if any
  if (is_vehicle_ahead(new_lane, predictions, id_car_ahead))
  {
    // let's evaluate predictions in the future...
    const Pose& car_ahead = predictions.at(id_car_ahead).back(); // grab future pose
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);

    if ( s_wrap2(car_ahead.s-ego_future.s) < m_buffer_distance) // closer than buffer, let's slow to match speed and find s
    {
      // back calc spd needed to reach desired s...
      float ego_future_desired_s = s_wrap(car_ahead.s - m_buffer_distance);
      Pose ego_tmp = ego_now;
      float spd_calc = s_wrap2(ego_future_desired_s-ego_tmp.s)/m_time_probe;
      float spd_old = ego_tmp.spd;
      ego_tmp.spd = min(spd_calc, spd_old);
      ego_future = m_rEgo.propagatePose(ego_tmp, m_time_probe); // with appropriate s reflecting new lane

cout << state << "(" << m_target_lane << ":" << new_lane << ")::New lane traffic speed = " << ego_future.spd
//     << ", (" << spd_calc << ", " << spd_old << ")"
//     << " (" << ego_future_desired_s << ", " << ego_future.s << ", " << ego_future_kl.s << ")"
//     << " [KLnow: (" << ego_now_kl.s << ", " << ego_now_kl.spd << "), future("
//                     << ego_future_kl.s << ", " << ego_future_kl.spd << ")] [PLCnow: ("
//                     << ego_tmp.s << ", " << ego_tmp.spd << "), future("
//                     << ego_future.s << ", " << ego_future.spd << ")]"
     << endl;
    }
    else // car ahead in new lane is still far away, use ref speed to calc distance
    {
//cout << state << " - car ahead still far!" << endl;
      ego_now.spd = m_ref_vel;
      ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
    }
  }
  else
  { // no cars ahead in new lane, use ref speed to calc distance
    ego_now.spd = m_ref_vel;
//cout << state << " - lane empty!" << endl;
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
  }

  // BUT while in PLCx mode, ego_future still need to follow car ahead (in own lane) speed!!!
  ego_future.spd = ego_future_kl.spd;

  return {ego_now, ego_future};
}


/*--------------------------
 * lane_change_trajectory()
  --------------------------*/
vector<Pose> PathPlanner::lane_change_trajectory(const FSM_state& state, const map<int,vector<Pose>>& predictions)
{
  int new_lane = m_target_lane;
  if (state==kFSM_LCR)
    new_lane += 1;
  else if (state==kFSM_LCL)
    new_lane -= 1;

  vector<Pose> trajectory;
  Pose ego_now = m_rEgo.getPose();
  float spd_now = ego_now.spd;
  Pose ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);

  //Check if a lane change is possible (traffic occupying spot).
  for (auto itr=predictions.begin(); itr!=predictions.end(); itr++)
  {
    // check if traffic is CURRENTLY next to ego in the new lane (assume 5m car length)
    const Pose& traffic_now = itr->second.front(); // current pose

    if ((traffic_now.lane==new_lane) &&
        (fabs(s_wrap2(traffic_now.s-ego_now.s)) < 5.0)) // overlap check
    {
      cout << state << " CAN'T: traffic blocking NOW!" << endl;
      return trajectory; // can't change lane, return empty trajectory
    }

    // check if traffic WILL BE next to ego in the new lane (assume 5m car length)
      // Once traffic lane change prediction is implemented,
      // this will catch traffic moving into our new lane
    const Pose& traffic_future = itr->second.back(); // future pose

    if ((traffic_future.lane==new_lane) &&
        (fabs(s_wrap2(traffic_future.s-ego_future.s)) < 5.0))
    {
      cout << state << " CAN'T: traffic blocking FUTURE!" << endl;
      return trajectory; // can't change lane yet, return an empty trajectory
    }
  }

  /* NOTE - This does NOT probe for speed or distances gained in the new lane!
   *        Should change this so ego_future indicates how 'good' this lane would be!!!
   */

  // lane change IS possible
  ego_now.lane = m_target_lane;
  ego_now.state = state;
  ego_now.spd = m_ref_vel+1; // use ref spd to calc new ego_future's S (+1 to make more attractive than PCLx)

  ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
  ego_future.lane = new_lane;
  ego_future.state = state;
  ego_future.spd = spd_now; // but keep current spd until after lane change.
  return {ego_now, ego_future};
}


/*------------------------
 * get_id_vehicle_ahead()
  ------------------------*/
//int PathPlanner::get_id_vehicle_ahead(int lane, const map<int,vector<Pose>>& predictions)
bool PathPlanner::is_vehicle_ahead(int lane, const map<int,vector<Pose>>& predictions, int& car_ahead_id)

{
  float max_s = m_rEgo.m_rTrack.m_max_s;
  float closest_dist_ahead = max_s/2;
  car_ahead_id = -1;

  for (auto itr=predictions.begin(); itr!=predictions.end(); itr++)
  {
    const Pose& traffic = itr->second.at(0); // traffic's current state
    if (traffic.lane == lane) // filter by lane
    {
      float traffic_s = traffic.s;
      float dist_ahead = s_wrap2(traffic.s-m_rEgo.getPose().s);

      // look for the car directly ahead of us
      if ((dist_ahead > 0) && (dist_ahead < closest_dist_ahead))
      {
        car_ahead_id = itr->first;
        closest_dist_ahead = dist_ahead;
      }
    }
  }

  if (car_ahead_id==-1)
    return false;
  else
    return true;
}


