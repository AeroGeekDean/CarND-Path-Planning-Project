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

using namespace std;
using namespace std::chrono;

// for convenience
using json = nlohmann::json;

// For keeping track of time to compute dt between data messages
bool sim_initialized = false;
steady_clock::time_point time_past;
duration<double> delta_t;

const double mph2ms = 0.45;
const double ms2mph = 1/mph2ms;

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

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // 181 WPTs...
  cout << "Loaded file: '" << map_file_ << "' with " << map_waypoints_x.size() << " WPTs." << endl;

  // start in lane 1;
  int lane = 1;

  // Have a reference velocity to target
  double ref_vel = 49.5; // [mph]

  h.onMessage([&ref_vel, &lane,
               &map_waypoints_x,
               &map_waypoints_y,
               &map_waypoints_s,
               &map_waypoints_dx, // Frenet d unit normal vector (x,y)
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws,
                                  char *data, size_t length,
                                  uWS::OpCode opCode) {
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

          // --- Time ---
          if (!sim_initialized) {
            time_past = steady_clock::now();
            sim_initialized = true;
          }

          // Compute elapsed time since last frame
          steady_clock::time_point time_now = steady_clock::now();
          delta_t = duration_cast<duration<double>>( time_now - time_past ); // dt in [sec]
          double dt = delta_t.count();
          time_past = time_now; // update past value

          double dt_ref = 0.02; // [sec], or 50Hz

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
          auto sensor_fusion = j[1]["sensor_fusion"]; // vector<vector<double>>

          int prev_size = previous_path_x.size();

       // ---- Above are INPUTS ----

//          cout << "dt " << dt << ", Hz " << 1/dt << endl;


       // ======== main algorithm goes here ==========


          // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
          // Later we will interpolate these waypoints with a spline and fill it in with more points taht control speed
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y, yaw states
          // either we will reference the starting point as where the car is, or at the previous path end point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if previous size is almost empty use the car as starting reference
          if (prev_size < 2)
          {
            // Use 2 points that make the path tangent to the car
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          // use the previous path's end point as starting reference
          else
          {

            // redefine reference starte as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            // Use two points that make the path tangent to the previous path's end point
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // In Frenet add evenly 30m spaced points ahead of the starting reference
          double d = 2+4*lane;
          vector<double> next_wp0 = getXY(car_s+30, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); i++)
          {
            // shift car reference angle to 0 deg
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            double delta_yaw = 0 - ref_yaw;
            ptsx[i] = (shift_x*cos(delta_yaw) - shift_y*sin(delta_yaw));
            ptsy[i] = (shift_x*sin(delta_yaw) + shift_y*cos(delta_yaw));
          }

          // create a spline
          tk::spline s;

          // set (x,y) points to the spline
          s.set_points(ptsx, ptsy);

          // Define the actual (x,y) points we weill use for the planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Start with all of the previous path points from the last frame
          for (int i=0; i<prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Calculate how to break up spline points so that we travel at our desired reference velocity
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;
          double ds = dt_ref*(ref_vel*0.45) * (target_x/target_dist); // s_distance per time step

          // Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
          for (int i=1; i<=(50-prev_size); i++)
          {
            x_add_on += ds;
            double x_point = x_add_on;
            double y_point = s(x_point);

            double x_tmp = x_point;
            double y_tmp = y_point;

            // rotate fom local to global coord
            x_point = (x_tmp*cos(ref_yaw) - y_tmp*sin(ref_yaw)) + ref_x;
            y_point = (x_tmp*sin(ref_yaw) + y_tmp*cos(ref_yaw)) + ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }





       // ---- Below are OUTPUTS ----

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
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
