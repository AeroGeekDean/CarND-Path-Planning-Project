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
#include "Vehicle.h"

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

  Vehicle ego;

  if (0) // transformation unit testing
  {
    ego.setPose(10, // car_x
                15, // car_y
                3,  // car_s,
                6,  // car_d,
                deg2rad(270.0), // car_yaw
                25.0 //car_speed
                );

    Vehicle::Coord wpt_g, wpt_b;
    wpt_g.x = 20;
    wpt_g.y = 15;
    wpt_b = ego.transformGlobal2Ego(wpt_g);
    wpt_g = ego.transformEgo2Global(wpt_b);
    cout << "(" << wpt_b.x << ", " << wpt_b.y << ")" << endl;
    cout << "(" << wpt_g.x << ", " << wpt_g.y << ")" << endl;
  }

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

  h.onMessage([&ego,
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

       // ---- Above are INPUTS ----

//          cout << "dt " << dt << ", Hz " << 1/dt << endl;


       // ======== main algorithm goes here ==========
          ego.setPose(car_x, car_y, car_s, car_d, deg2rad(car_yaw), car_speed);

          // containers for algorithm output
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // ---- create a local smoothed WPT sub-database ----
          int num_wpts = map_waypoints_x.size();
          int next_wpt_idx = NextWaypoint(car_x, car_y, deg2rad(car_yaw), map_waypoints_x, map_waypoints_y);

          vector<double> local_wpts_xb;
          vector<double> local_wpts_yb;
          vector<double> local_wpts_s;

          Vehicle::Coord wpt_g; // global coord
          Vehicle::Coord wpt_b; // body coord

          // based on 2 WPTs beind and 2 WPTs in front
          for (int i=next_wpt_idx-2; i<next_wpt_idx+2; i++)
          {
            int j = (i+num_wpts) % num_wpts; // handle both negative and index looping around end
            wpt_g.x = map_waypoints_x[j];
            wpt_g.y = map_waypoints_y[j];
            wpt_b = ego.transformGlobal2Ego(wpt_g); // transform to ego body coord axis
            local_wpts_xb.push_back( wpt_b.x );
            local_wpts_yb.push_back( wpt_b.y );
            local_wpts_s.push_back( map_waypoints_s[j] );
          }

          // create and setup the spline
          tk::spline local_body_wpt_spline_sx; // s->x
          local_body_wpt_spline_sx.set_points( local_wpts_s, local_wpts_xb );

          tk::spline local_body_wpt_spline_sy; // s->y
          local_body_wpt_spline_sy.set_points( local_wpts_s, local_wpts_yb );


          // re-use all the remaining prev trajectory points
//          int prev_path_size = previous_path_x.size();
          int prev_path_size = 0;
          //          for (int i=0; i<prev_path_size; i++) {
//            next_x_vals.push_back(previous_path_x[i]);
//            next_y_vals.push_back(previous_path_y[i]);
//          }

          double next_wpt_s;
          if (prev_path_size==0)
            next_wpt_s = car_s;
          else
            next_wpt_s = end_path_s;

          vector<double> local_smoothed_wpts_x;
          vector<double> local_smoothed_wpts_y;
          vector<double> local_smoothed_wpts_s;

          double speed_mph = 30.0; // [mph]

          // ---- create smooth wpt map for 'look ahead' segment ----
          double look_ahead_time = 2.0; // [sec]
          int    look_ahead_slices = 10;
          double ds_look_ahead = look_ahead_time * mph2ms*speed_mph / (double)look_ahead_slices; // [m]

          double temp_s;
          for (int i=0; i<look_ahead_slices; i++)
          {
            temp_s = next_wpt_s + i*ds_look_ahead;
            wpt_b.x = local_body_wpt_spline_sx(temp_s);
            wpt_b.y = local_body_wpt_spline_sy(temp_s);
            wpt_g = ego.transformEgo2Global(wpt_b);
            local_smoothed_wpts_s.push_back( temp_s );
            local_smoothed_wpts_x.push_back( wpt_g.x );
            local_smoothed_wpts_y.push_back( wpt_g.y );
          }


          double dt_ideal = 0.02;  // [sec]
          double ds = dt_ideal* mph2ms*speed_mph; // [m]

          double next_wpt_d = 6.0; // lane is 4m wide

          cout << car_yaw << ", ";

          // fill to 50 size
          vector<double> next_wpt;
          for (int i = 0; i < (50-prev_path_size); i++) {
            next_wpt_s += ds;
            cout << next_wpt_s << ", ";
            next_wpt = getXY(next_wpt_s, next_wpt_d,
                             local_smoothed_wpts_s, local_smoothed_wpts_x, local_smoothed_wpts_y);
//                             map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals.push_back(next_wpt[0]);
            next_y_vals.push_back(next_wpt[1]);
          }
          cout << endl;



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
