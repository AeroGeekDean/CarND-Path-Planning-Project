/*
 * PathPlanner.cpp
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

#include "PathPlanner.h"
#include "EgoVehicle.h"
#include "UtilFunctions.h"
#include "spline.h"
#include <iostream>

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
                                 double end_s, double end_d)
{
  m_prev_path_x     = path_x;
  m_prev_path_y     = path_y;
  m_prev_path_size  = path_x.size();

  if (m_prev_path_size==0)
  { // these are undefined/zero when there's is no prev path (simulator 1st frame)
    // thus assigning ego's current pose instead
    m_end_path_s = m_rEgo.getPose().s;
    m_end_path_d = m_rEgo.getPose().d;
  }
  else
  {
    m_end_path_s = end_s;
    m_end_path_d = end_d;
  }
}

/*------------------------
 * generate_trajectory()
 *
 * This method performs trajectory generation,
 * after behavior planning has completed.
  ------------------------*/
vector<Pose> PathPlanner::generate_trajectory(int lane, float v_in, int num_pp_pts)
{
  /* INPUTS:
   *
   *  int   lane       - the target lane the trajectory should head towards
   *  float v_in       - speed command input [m/s]. Could be a discontinuous step
   *  float traj_time  - look ahead time [sec] the trajectory is build out to
   *  int   num_pp_pts - number of points to re-use from previous trajectory
   *                     (Default = -1, which is to re-use ALL the points)
   *
   * OUTPUT:
   *
   *  vector<Pose> - A trajectory defined by the x,y points (only) of each pose
   *                 within the container
   *
   * Overall strategy:
   *
   * Create a set of widely spaced (x,y) 5 waypoints. The first 2 points define
   * the starting reference location and orientation, then 3 more evenly-spaced
   * points far ahead of the vehicle.
   *
   * A spline is then fitted to these 5 points, and finer spaced points are interpolated
   * for the vehicle trajectory. The spacing of these trajectory points will define
   * the vehicle speed (and acceleration, jerk).
   *
   * Because of the asynchronous nature of the simulator and controller process,
   * we do not know, a priori, the actual process frame time and i/o delays. Thus
   * in order to keep continuity with previous frames, the simulator provides back
   * the REMAINING trajectory points from the previous frame. We re-use some of
   * these previous trajectory points in the construction of new trajectory points.
   * The number of previous points could be selected via the method parameters.
   *
   */

  int num_steps = (int)ceil(m_time_traj/m_dt_ref); // calc # of pts to build the trajectory with

  vector<double> ptsx, ptsy; // containers for the widely spaced points

  int ref_ppath_size; // reference previous path size
  if ((num_pp_pts == -1) ||             // Default: use all prev points
      (num_pp_pts > m_prev_path_size))  // requesting too many points
    ref_ppath_size = m_prev_path_size;
  else
    ref_ppath_size = num_pp_pts;

  // --- Determine starting reference pose ---
  // use either ego current state, or end of previous-path-points

  double ref_x;
  double ref_y;
  double ref_s;
  double ref_yaw;
  double ref_spd;
  double ref_d = 2 + 4*lane; // each lane is 4[m] wide, center of lane is 2[m] from edge

//  double ref_accel;

  if (ref_ppath_size < 3)  // insufficient points, use ego as starting reference
  {
    ref_x   = m_rEgo.getPose().x;
    ref_y   = m_rEgo.getPose().y;
    ref_s   = m_rEgo.getPose().s;
    ref_yaw = m_rEgo.getPose().yaw;
    ref_spd = m_rEgo.getPose().spd;
//    ref_accel = 0.0;

    // find another point slight behind the ego, to align spline to the car's heading
    double prev_car_x = ref_x - cos(ref_yaw);
    double prev_car_y = ref_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(ref_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(ref_y);
  }
  else  // use prev path end for starting reference
  {
    ref_x = m_prev_path_x[ref_ppath_size-1];
    ref_y = m_prev_path_y[ref_ppath_size-1];

    double ref_x_prev = m_prev_path_x[ref_ppath_size-2];
    double ref_y_prev = m_prev_path_y[ref_ppath_size-2];

    ref_yaw = atan2( (ref_y-ref_y_prev), (ref_x-ref_x_prev) );
    ref_spd = distance(ref_x, ref_y, ref_x_prev, ref_y_prev)/m_dt_ref;

    // ref_accel is needed for 2nd-order lag filter implementation... (approach abandoned)
//    double ref_x_pprev = m_prev_path_x[ref_ppath_size-3];
//    double ref_y_pprev = m_prev_path_y[ref_ppath_size-3];
//    double dist2 = distance(ref_x_prev, ref_y_prev, ref_x_pprev, ref_y_pprev);
//    ref_accel = (ref_spd - dist2/m_dt_ref)/m_dt_ref;

    ref_s = m_rEgo.m_rTrack.getFrenet(ref_x, ref_y, ref_yaw).at(0);

    // Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // In Frenet add evenly spaced points ahead at 30, 60, 90[m], from the ref point
  for (int ahead_s : {30, 60, 90})
  {
    vector<double> next_wp = m_rEgo.m_rTrack.getXY(ref_s+ahead_s, ref_d); // convert to global x,y
    ptsx.push_back( next_wp.at(0) );
    ptsy.push_back( next_wp.at(1) );
  }

  // TRANSFORM from global to local coordinate (using ref_x/_y/_yaw as local origin)
  // NOTE: We're re-using ptsx and ptsy. Need to transform back to global coord later!
  for (int i=0; i<ptsx.size(); i++)
  {
    vector<double> out = global2local(ptsx.at(i), ptsy.at(i), ref_x, ref_y, (0-ref_yaw));
    ptsx.at(i) = out.at(0);
    ptsy.at(i) = out.at(1);
  }

  // create and set (x,y) points to a spline
  tk::spline s_curve;
  s_curve.set_points(ptsx, ptsy);

  vector<Pose>   trajectory; // trajectory output container

  // Start with portion of the previous path points from the last frame
  for (int i=0; i<ref_ppath_size; i++)
  {
    Pose p;
      p.x = m_prev_path_x[i];
      p.y = m_prev_path_y[i];
    trajectory.push_back(p); // these are in global (x,y) coord already
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
   *  With the associated 2nd order (Laplace transform) transfer function as:
   *
   *    Vout/Vin = (gain_a*gain_j) / ( S^2 + gain_j*S + gain_a*gain_j )
   *
   *  where:
   *    Omega_n (natural frequency)   = sqrt( gain_a*gain_j )
   *    zeta (damping ratio)          = 0.5 * gain_j / Omega_n
   *
   *  Thus with:
   *    gain_a = 0.403 [1/sec],       gain_j = 0.9 [1/sec]
   *
   *  The filter dynamics are:
   *    Omega_n = 0.602 [rad/sec]
   *    Zeta    = 0.747 [Non-Dimensional]   <--- under-damped, oscillatory!!!
   *
   *  However, I was having trouble debugging out the long-period speed oscillation
   *  and ending up spending too much time on controller design, instead of project
   *  path planning design.
   *
   *  THUS ABANDONED THIS APPROACH, FOR NOW.
   *
   *  I INSTEAD implemented a much simpler "LIMITED ACCELERATION SPEED CONTROLLER".
   *
   *  By limiting the max acceleration to 5.0 [m/s^2], and implementing it as a
   *  max-speed-change-per-frame limit (m_dv_accel_lim) of 0.1 [m/s] (= 5.0*0.02)
   *  This yields a max jerk of only 2.5 [m/s^3] for a large step speed input...
   *  ONLY when a large step speed input is immediately followed by a large step
   *  in opposite direction, does the max jerk reach 10.0 [m/s^3].
   *
   *  ---- Below contains (commented out) code associated with the original design.
   *  ---- Preserving, so I can come back to them later.
   */

//  double a_prev = ref_accel;  // acceleration at the end of previous path points
  double v_prev = ref_spd;      // velocity at the end of previous path points

  /* SPEED CONTROLLER gain_a (below) is chosen based on:
   * - Estimated max step delta velocity command is 50 [mph] = 22.35 [m/s],
   * - accel_lim = 10 [m/s^2] -> allow 90% of that, so 9 [m/s^2]
   * - hence: 9/22.35 = 0.403
   */
//  float gain_a = 0.403; // accel_cmd per velocity_error, [1/sec],

    /* Similarly, SPEED CONTROLLER gain_j (below) is chosen based on:
     * - max accel = 10.0 [m/s^2],
     * - jerk_lim = 10 m/s^3 -> allow 90% of that, so 9 [m/s^3]
     * - hence: 9/10 = 0.9
     */
//  float gain_j = 0.9;   // jerk_cmd per acceleration_error, [1/sec]

//  double j; // jerk, [m/s^3]

  double dv;  // delta_speed per frame
  double ds;  // delta_frenet_s per frame
  double x = 0.0, y;
  double slope; // slope of dy/dx
  double x_prev = 0.0; // origin of body axis
  double y_prev = 0.0;

  /* CONVERSION OF SPEED & X_STEP, FROM FRENET-S DIRECTION TO BODY-X DIRECTION....
   * Since our given speed is in Frenet Sdirection, we need to do a little bit
   * of trigonometry to find our step size in the local X-direction. S & X are
   * co-aligned right at the vehicle, but will diverge as the trajectory forward
   * curves & bends. We'll use the previous step 'slope' to predict each future
   * step 'slope'. Furthermore, We'll also use Trigonometry Identity equation
   * to calculate `cosine_ratio` below, so as to avoid using the trigonometry
   * cos() & atan2() functions, which are computationally costly.
   */
  double cosine_ratio = 1.0; // cosine of slope angle, ie: cos(atan2(dy/dx))

  // remaining # of trajectory points to build
  int remaining_steps = max( (num_steps-ref_ppath_size), 0);

  for (int i=0; i<remaining_steps; i++)
  {
//--- These are the 2nd order lag filter speed controller code ---
//    Saving them here for future
//
//    j = gain_j*(gain_a*(vref_in - v_prev) - a_prev);
//    a_prev += j*m_dt_ref;
//    v_prev += a_prev*m_dt_ref;
//    v_prev = min((v_prev + a_prev*dt_in), (double)m_ref_vel);

    // rate limit how much delta_v is permitted per frame (ie: acceleration limiting)
    dv = max(min((v_in - v_prev), m_dv_accel_lim), -m_dv_accel_lim);
    v_prev += dv;

    /* This below causes the simulator to draw the LAST trajectory path ball out ahead
     * to m_time_probe [sec] distance. This is how far ahead the PathPlanner is looking!
     * Keep in mind that all the other traffic would have moved ahead as well.
     */
    float dt;
    if ((num_pp_pts!=-1) && // Don't do this if re-using entire past trajectory!
        (i==(remaining_steps-1))) // detect last step in for-loop
      dt = m_time_probe - (m_time_traj - m_dt_ref); // make dt reach out to where m_time_probe would been at
    else
      dt = m_dt_ref;

    ds = v_prev * dt;
    x += cosine_ratio*ds;  // find dx based on trajectory slope
    y = s_curve(x);

    // convert (x,y) back to global
    vector<double> out = local2global(x, y, ref_x, ref_y, ref_yaw);
    Pose p;
      p.x = out[0];
      p.y = out[1];
    trajectory.push_back(p);

/*  Actual cosine_ratio = cos( atan2((y_point-y_point_prev), (x_point-x_point_prev)) );
 *  Below is simpler formula using trig identities
 */
    slope = (y-y_prev)/(x-x_prev);
    cosine_ratio = 1/sqrt(1+slope*slope);  // find new value for next frame
    x_prev = x; // save previous values
    y_prev = y;
  }

//  cout << "TrajCalc_v: ["; // for debugging
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
 *
 * This method performs the behavior planning.
   -------------------*/
vector<Pose> PathPlanner::chooseNextState(const map<int,vector<Pose>>& predictions)
{
  /*
   * This method is modeled after the Behavior Planning pseudocode classroom concept
   *
   * INPUT:
   *
   * A predictions map. This is a map using vehicle id as keys with a vector of
   * predicted vehicle poses as values. The first item in the pose vector represents
   * the vehicle pose at the current time step. The second item represents the vehicle
   * pose one time step in the future. The last item represents the vehicle pose at the
   * end of the look ahead probe time.
   *
   * OUTPUT:
   *
   * The the best (lowest cost) trajectory for the ego vehicle corresponding to
   * the next ego vehicle state.
   *
   * HELPER FUNCTIONS:
   *
   * 1. successor_states() - Uses the current state to return a vector of possible
   * successor states for the finite state machine. The states are enums.
   *
   * 2. make_predictive_poses(FMS_state state,
   *                          const map<int, vector<Pose>>& predictions) -
   * Returns a vector of poses representing a vehicle trajectory, given a state and
   * predictions. Note that pose vectors might have size 0 if no possible trajectory
   * exists for the state.
   *
   * 3. calculate_cost(Pose pose, const map<int, vector<Pose>>& predictions,
   *                   const vector<Pose>& trajectory) -
   * computes the cost for a predictive pose (trajectory).
   */


  // Generate predictive poses associated with each state, then calculate their associated cost.

  vector<FSM_state> states = possibleSuccessorStates();

  vector<float> costs;
  vector<vector<Pose>> trajectories; // each trajectory contains only {present_pose, future_pose}

  for (auto state : states)
  {
    vector<Pose> trajectory = make_predictive_poses(state, predictions);
    if (trajectory.size() != 0)
    {
      float cost = calculate_cost(m_rEgo.getPose(), predictions, trajectory);
      costs.push_back(cost);
      trajectories.push_back(trajectory);
    }
  }

  // Find and return the lowest cost trajectory
  vector<float>::iterator best_cost_itr = min_element(begin(costs), end(costs));
  int best_idx = std::distance(begin(costs), best_cost_itr);

  // extract final pose of best trajectory
  const Pose& p = trajectories.at(best_idx).back();

  if (m_state != p.state) // a state change occured, let's cout it
  {
    cout << "Spd: " << p.spd << ", S: " << p.s << ", State: " << p.state << "[" << p.lane << "]: ";
    for (int i=0; i<costs.size(); i++) {
      if (i==best_idx)
        cout << trajectories.at(i).back().state << "(" << (costs.at(i)-*best_cost_itr) << "), ";
      else
        cout << trajectories.at(i).back().state << " " << (costs.at(i)-*best_cost_itr) << " , ";
    } cout << endl;
    cout<<endl;
  }

  // update PathPlanner FSM's state & lane
  m_state       = p.state;
  m_target_lane = p.lane;

  // build the actual trajectory, (re-using only 10 pts from prev path)
  return generate_trajectory(p.lane, p.spd, 10);
}

/*------------------
 * calculate_cost()
  ------------------*/
float PathPlanner::calculate_cost(Pose pose,
                                  const map<int,vector<Pose>>& predictions,
                                  const vector<Pose>& trajectory)
{
  /*
   * Cost is simply how far ahead the trajectory makes progress.
   */
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
   * Provides the possible next states given the current state for the FSM
   */
  vector<FSM_state> states;
  states.push_back(kFSM_KL); // KL is always a viable state

  FSM_state state = m_state;
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

/*-------------------------
 * make_predictive_poses()
  -------------------------*/
vector<Pose> PathPlanner::make_predictive_poses(FSM_state state, const map<int,vector<Pose>>& predictions)
{
  /*
   * Given a possible next state, generate the appropriate predictive poses (trajectory)
   * to realize the next state.
   */
  vector<Pose> predictive_poses;
  switch (state)
  {
    case kFSM_CS:
      predictive_poses = constant_speed_poses();  // for traffic vehicle only
      break;
    case kFSM_KL:
      predictive_poses = keep_lane_poses(predictions);
      break;
    case kFSM_LCL:
    case kFSM_LCR:
      predictive_poses = lane_change_poses(state, predictions);
      break;
    case kFSM_PLCL:
    case kFSM_PLCR:
      predictive_poses = prep_lane_change_poses(state, predictions);
      break;
    default:
      break;
  }
  return predictive_poses;
}

/*------------------------
 * constant_speed_poses()
  ------------------------*/
vector<Pose> PathPlanner::constant_speed_poses()
{
  // Continue on ego's current lane and speed
  Pose ego_now = m_rEgo.getPose();
  ego_now.state = kFSM_CS;

  return {m_rEgo.getPose(), m_rEgo.propagatePose(ego_now, m_time_probe)};
}

/*--------------------
 * keep_lane_poses()
  --------------------*/
vector<Pose> PathPlanner::keep_lane_poses(const map<int,vector<Pose>>& predictions)
{
  // Continue on ego's current lane
  // But if there're traffic ahead, adjust speed to 'match' with them.
  // 'Match' == same speed but some distance behind the lead vehicle

  const float kCollision_avoidance_gain = 1.0; // [m/s per m], how much to slow down
                                               //         to avoid hitting car ahead
  int id_car_ahead;

  Pose ego_now = m_rEgo.getPose();
  ego_now.state = kFSM_KL;      // set the FSM state
  ego_now.lane = m_target_lane; // follow last target lane, as ego might be in maneuver transient

  Pose ego_future;

  // Find the car immediately ahead of us, if any
  if (is_vehicle_ahead(m_target_lane, predictions, id_car_ahead))
  {
    // check distance gap in the future...
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
    const Pose& car_ahead_future = predictions.at(id_car_ahead).back();

    if ( s_wrap2(car_ahead_future.s-ego_future.s) < m_buffer_distance ) // slow to match speed
    {
      // calc distance needed to maintain future distance buffer...
      float ego_future_desired_s = s_wrap(car_ahead_future.s - m_buffer_distance);
      float spd_calc = s_wrap2(ego_future_desired_s-ego_now.s)/m_time_probe;
      ego_now.spd = min(spd_calc, (float)ego_now.spd); // prevent speeding up

      // check distance gap at current time...
      const Pose& car_ahead_now = predictions.at(id_car_ahead).front();
      float buffer_encroachment_dist = m_buffer_distance - s_wrap2(car_ahead_now.s-ego_now.s); // (+) encroach
      if ( buffer_encroachment_dist > 0 )
      { // closer than buffer, actively reduce speed further
        float spd_reduction = max(buffer_encroachment_dist*kCollision_avoidance_gain, 0.0f);
        ego_now.spd -= spd_reduction;
      }

      ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
      //cout << "KL - car ahead too close! New speed = " << ego_future.spd << endl; // debug output
    }
    else // car ahead is far away, maintain ref_vel speed
    {
      ego_now.spd = m_ref_vel;
      ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
      //cout << "KL - car ahead still far!" << endl; // debug output
    }
  }
  else
  { // no cars ahead, maintain ref_vel speed
    ego_now.spd = m_ref_vel;
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
    //cout << "KL - lane empty!" << endl; // debug output
  }
  return {ego_now, ego_future};
}


/*---------------------------
 * prep_lane_change_poses()
  ---------------------------*/
vector<Pose> PathPlanner::prep_lane_change_poses(const FSM_state& state,
                                                 const map<int,vector<Pose>>& predictions)
{
  int new_lane = m_target_lane;
  if (state==kFSM_PLCR)
    new_lane += 1;
  else if (state==kFSM_PLCL)
    new_lane -= 1;

  int id_car_ahead;

  // we'll need these later
  const vector<Pose>& kl_data = keep_lane_poses(predictions);
  const Pose& ego_now_kl = kl_data.front();
  const Pose& ego_future_kl = kl_data.back();

  Pose ego_now = m_rEgo.getPose();
  ego_now.state = state;        // set the FSM state
  ego_now.lane = m_target_lane; // follow last target lane, as ego might be in maneuver transient

  Pose ego_future;

  // Find the car that is ahead of us in new lane, if any
  if (is_vehicle_ahead(new_lane, predictions, id_car_ahead))
  {
    // check distance gap in the future...
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
    const Pose& car_ahead = predictions.at(id_car_ahead).back();

    if ( s_wrap2(car_ahead.s-ego_future.s) < m_buffer_distance) // let's slow to match speed and find s
    {
      // calc distance needed to maintain disance buffer...
      float ego_future_desired_s = s_wrap(car_ahead.s - m_buffer_distance);
      float spd_calc = s_wrap2(ego_future_desired_s-ego_now.s)/m_time_probe;
      ego_now.spd = min(spd_calc, (float)ego_now.spd);  // prevent speeding up
      ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
//cout << state << "(" << m_target_lane << ":" << new_lane << ")::New lane traffic speed = " << ego_future.spd << endl; // debug output
    }
    else // car ahead in new lane is still far away, use ref_vel to calc distance
    {
      ego_now.spd = m_ref_vel;
      ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
      //cout << state << " - car ahead still far!" << endl;
    }
  }
  else
  { // no cars ahead in new lane, use ref_vel to calc distance
    ego_now.spd = m_ref_vel;
    ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
    //cout << state << " - lane empty!" << endl;
  }

  // BUT while in PLCx mode, ego_future still needs to follow car ahead (in own lane) speed!!!
  // Since ego_future.spd is used for speed control of the active trajectory
  ego_future.spd = ego_future_kl.spd;

  return {ego_now, ego_future};
}


/*----------------------
 * lane_change_poses()
  ----------------------*/
vector<Pose> PathPlanner::lane_change_poses(const FSM_state& state, const map<int,vector<Pose>>& predictions)
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
//      cout << state << " CAN'T: traffic blocking NOW!" << endl; // debug output
      return trajectory; // can't change lane, return empty trajectory
    }

    // check if traffic WILL BE next to ego in the new lane (assume 5m car length)
      // Once traffic lane change prediction is implemented,
      // this will catch traffic moving into our new lane
    const Pose& traffic_future = itr->second.back(); // future pose

    if ((traffic_future.lane==new_lane) &&
        (fabs(s_wrap2(traffic_future.s-ego_future.s)) < 5.0))
    {
//      cout << state << " CAN'T: traffic blocking FUTURE!" << endl; // debug output
      return trajectory; // can't change lane yet, return an empty trajectory
    }
  }

  // lane change IS possible
  ego_now.lane = m_target_lane;
  ego_now.state = state;
  ego_now.spd = m_ref_vel+1; // use ref_vel to calc new ego_future's S
                             // (+1 to make it slightly more attractive than PCLx)

  ego_future = m_rEgo.propagatePose(ego_now, m_time_probe);
  ego_future.lane = new_lane;
  ego_future.state = state;
  ego_future.spd = spd_now; // but keep current speed until after lane change.
  return {ego_now, ego_future};
}


/*------------------------
 * get_id_vehicle_ahead()
  ------------------------*/
bool PathPlanner::is_vehicle_ahead(int lane, const map<int,vector<Pose>>& predictions, int& car_ahead_id)
{
  float closest_dist_ahead = m_rEgo.m_rTrack.m_max_s/2; // max possible dist ahead
  car_ahead_id = -1;

  for (const auto& kv : predictions)
  {
    const Pose& traffic = kv.second.at(0); // traffic's current state
    if (traffic.lane == lane) // filter by lane
    {
      float traffic_s = traffic.s;
      float dist_ahead = s_wrap2(traffic.s - m_rEgo.getPose().s);

      // look for the car directly ahead of us
      if ((dist_ahead > 0) && (dist_ahead < closest_dist_ahead))
      {
        car_ahead_id = kv.first;
        closest_dist_ahead = dist_ahead;
      }
    }
  }

  if (car_ahead_id==-1)
    return false;
  else
    return true;
}


