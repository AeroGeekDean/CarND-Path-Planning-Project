#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "UtilFunctions.h" // <--- moved many global helper functions here
#include "spline.h"
#include "Track.h"
#include "TrafficManager.h"
#include "EgoVehicle.h"

using namespace std::chrono;

using std::min;
using std::cout;
using std::endl;
using std::string;

// for convenience
using json = nlohmann::json;

// For keeping track of time to compute dt between data messages
bool sim_initialized = false;
steady_clock::time_point time_past;
duration<double> delta_t;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  cout << std::fixed << std::setprecision(3);

  int lane = 1;
  float ref_vel    = 49.0*mph2ms; // [mph]

  float dt_ref     = 0.02; // [sec], or 50Hz. frame rate used by simulator
  float time_traj  = 1.0;  // [sec] look ahead time used to build vehicle trajectory for vehicle control
  float time_probe = 3.0;  // [sec] Look-ahead time used for traffic prediction & ego planning

  float buffer_dist = 10.0; // [m] distance to keep back from car ahead.
                            // Assume 1 car length = 5m, thus 10m gives us 1 car spacing from bumper-to-bumper

  float accel_lim  = 5.0; // [m/s^2], rubic limit = 10.0
  float jerk_lim   = 5.0; // [m/s^3], rubic limit = 10.0


  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  // --- create major components ---

  Track track; // <--- The track class holds knowledge about the track,
              //        including waypoint map and Frenet<>XY conversions
  track.m_num_lanes_available = 3;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  string line;
  double x;
  double y;
  float  s;
  float  d_x;
  float  d_y;
  while (getline(in_map_, line)) {
  	std::istringstream iss(line);
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	track.map_waypoints_x.push_back(x);
  	track.map_waypoints_y.push_back(y);
  	track.map_waypoints_s.push_back(s);
  	track.map_waypoints_dx.push_back(d_x);
  	track.map_waypoints_dy.push_back(d_y);
  }
  track.processWpts();
  track.m_max_s = max_s;

  TrafficManager traffic_mgr(track);  // <--- The TrafficManager class holds knowledge of the traffic vehicles,
                                      //      and their predictions. Each traffic vehicle is an instance of the
                                      //      Vehicle class. The Vehicle class holds the pose of the vehicle,
                                      //      and knows how to propagate that pose forward in time.
  traffic_mgr.m_time_probe = time_probe;

  EgoVehicle ego(track); // <--- The EgoVehicle class derives from the Vehicle class, and contains a PathPlanner.
                         //      The PathPlanner class is the bulk of the "brain" for this project. The planner's
                         //      algorithm strategy is based on the Behavior planning pseudocode from the class room
                         //      Long url link below.
//      https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/56274ea4-277d-4d1e-bd95-ce5afbad64fd/concepts/e9f08f5f-0b8f-488d-8940-45bc474b4913

  ego.m_PathPlanner.m_dt_ref = dt_ref;
  ego.m_PathPlanner.m_time_traj = time_traj;
  ego.m_PathPlanner.m_time_probe = time_probe;
  ego.m_PathPlanner.m_target_lane = lane;
  ego.m_PathPlanner.m_ref_vel = ref_vel;
  ego.m_PathPlanner.m_buffer_distance = buffer_dist;
  ego.m_PathPlanner.m_dv_accel_lim = accel_lim*dt_ref; // set accel limit, as max delta-v per time step
  ego.m_PathPlanner.jerk_lim = jerk_lim*dt_ref;  // set jerk limit, as max delta-a per time step

  // 181 WPTs...
  cout << "Loaded file: '" << map_file_ << "' with " << track.m_num_wpts << " WPTs." << endl;

  h.onMessage([&track, &traffic_mgr, &ego, &ref_vel]
               (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();

        if (event == "telemetry") {

          // j[1] is the data JSON object

        	// Main car's localization Data
          double car_x = j[1]["x"];     // [m]
          double car_y = j[1]["y"];     // [m]
          double car_s = j[1]["s"];     // [m]
          double car_d = j[1]["d"];     // [m]
          double car_yaw = j[1]["yaw"]; // [deg] (+) yaw left!
          double car_speed = j[1]["speed"]; // [mph]

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // data format: for each car [id, x, y, vx, vy, s, d]
          auto sensor_fusion = j[1]["sensor_fusion"]; // vector<vector<double>>

          // --- Time ---
          if (!sim_initialized) {
            time_past = steady_clock::now();
            sim_initialized = true;
          }

          // Compute elapsed time since last frame
          steady_clock::time_point time_now = steady_clock::now();
          delta_t = duration_cast<duration<double>>( time_now - time_past ); // dt in [sec]
          double dt_actual = delta_t.count();
          time_past = time_now; // update past value
//          cout << "dt_actual " << dt_actual << ", Hz " << 1/dt_actual << endl;

       // ---- Above are INPUTS ----

          // ======== main (per frame) algorithm goes here ==========

          /*---------------------
           * Traffic Predictions
            ---------------------*/
          traffic_mgr.updateTraffic(sensor_fusion);
          traffic_mgr.predict();

          /*------------------
           * Update Ego Data
            -----------------*/
          Pose p;
            p.x = car_x;
            p.y = car_y;
            p.s = car_s;
            p.d = car_d;
            p.yaw = deg2rad(car_yaw);
            p.spd = min((mph2ms*car_speed), (double)ref_vel); // feedback speed contains noise. setting upper limit
          ego.updatePose(p);

          ego.m_PathPlanner.updatePrevPath( previous_path_x, previous_path_y, end_path_s, end_path_d );

          /*------------------
           * Run Path Planner
            -----------------*/
          vector<Pose> trajectory_output = ego.m_PathPlanner.chooseNextState( traffic_mgr.m_predictions );

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (const Pose& pose : trajectory_output)
          {
            next_x_vals.push_back(pose.x);
            next_y_vals.push_back(pose.y);
          }

       // ---- Below are OUTPUTS ----

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        } // if (event == "telemetry")
      } // if (s != "")...
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // if "42" at start of socket data...
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
