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
#include "Pose.h"

using namespace std;
using namespace std::chrono;

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

  int lane = 1;
  double ref_vel    = 49.0; // [mph]
  double dt_ref     = 0.02; // [sec], or 50Hz
  double accel_lim  = 5.0; // [m/s^2], rubic limit = 10.0
  double jerk_lim   = 5.0; // [m/s^3], rubic limit = 10.0


  // create major components
  Track track;
  TrafficManager traffic_mgr(track);
  EgoVehicle ego(track);

  ego.m_PathPlanner.m_dt = dt_ref;
  ego.m_PathPlanner.accel_lim = accel_lim*dt_ref; // set accel limit, as max delta-v per time step
  ego.m_PathPlanner.jerk_lim = jerk_lim*dt_ref;  // set jerk limit, as max delta-a per time step

  track.m_num_lanes_available = 3;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
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

  // 181 WPTs...
  cout << "Loaded file: '" << map_file_ << "' with " << track.m_num_wpts << " WPTs." << endl;

  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  h.onMessage([&track, &traffic_mgr, &ego, &ref_vel, &lane, &max_s]
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

          int prev_size = previous_path_x.size();

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



       // ======== main algorithm goes here ==========

          /*---------------------
           * Traffic Predictions
            ---------------------*/

          traffic_mgr.updateTraffic(sensor_fusion);
          traffic_mgr.predict();

          double car_s_future;
          if (prev_size > 0)
            car_s_future = end_path_s; // note: this will impact later code!!!

          /*------------------
           * Update Ego Data
            -----------------*/

          Pose p;
            p.x = car_x;
            p.y = car_y;
            p.s = car_s;
            p.d = car_d;
            p.yaw = deg2rad(car_yaw);
            p.spd = mph2ms*car_speed;
          ego.updatePose(p);

          ego.m_PathPlanner.updatePrevPath(previous_path_x, previous_path_y, end_path_s, end_path_d);

//          vector<Pose> traj_out = ego.m_PathPlanner.chooseNextState( traffic_mgr.m_predictions );

//          ego.realizeNextState(); // this would be trajectory generation to pass to simulator

//          cout << "number of sensed vehicles: " << sensor_fusion.size() << " {";

          /*-------------------
           * Behavior Planning
            -------------------*/

          bool too_close = false;

          // find ref_v to use
          for (int i=0; i<sensor_fusion.size(); i++)
          {
            string ahead;
            double traffic_s = sensor_fusion[i][5];
            double dist_ahead = traffic_s - car_s;
            while (dist_ahead >   (max_s/2)) dist_ahead -= max_s; // constraint to -(max_s/2) < X <= (max_s/2)
            while (dist_ahead <= -(max_s/2)) dist_ahead += max_s;
            if (dist_ahead > 0) ahead = "*";
            else                ahead = "_";

            // is car in my lane?
            float d = sensor_fusion[i][6]; // [m]?
            if ((2+4*lane -2) < d && d < (2+4*lane +2))
            {
              double vx = sensor_fusion[i][3];  // [m/s]?
              double vy = sensor_fusion[i][4];  // [m/s]?
              double check_speed = sqrt(vx*vx+vy*vy); // [m/s]  // Note: this only checks straight line in global coord, not aware of lane curves!!!
              double check_car_s = sensor_fusion[i][5]; // [m]?

              check_car_s += ((double)prev_size* 0.02*check_speed); // if using previous points can project s value outwards in time

              if (check_car_s > car_s_future) // car ahead of me?
              {
                if ((check_car_s - car_s_future) < 30) // < 30m ahead? (didn't account for track S-wrap-around)
                {
                  ref_vel = ms2mph*check_speed; // [mph]
                  cout << "speed limited: traffic speed = " << ref_vel << endl;
  //                 too_close = true;
                   if (lane > 0)
                     lane -= 1;
                }
              }
//              cout << "<" << ahead << sensor_fusion[i][0] << ">, ";
            }
            else
            {
//              cout << " " << ahead << sensor_fusion[i][0] << " , ";
            }
          }
//          cout << endl;

          /*-----------------------
           * Trajectory Generation
            -----------------------*/

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<Pose> output = ego.m_PathPlanner.getTrajectoryOutput(lane, ref_vel*mph2ms, 10);
          for (int i=0; i<output.size(); i++)
          {
            next_x_vals.push_back(output.at(i).x);
            next_y_vals.push_back(output.at(i).y);
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
        else
          cout << event << endl;

      } // if (s != "")...
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        cout << "manual: " << endl;
//        cout << "manual: " << j[1] << endl;
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
