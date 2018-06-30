/*
 * Track.h
 *
 *  Created on: Jun 28, 2018
 *      Author: deanliu
 */

#ifndef TRACK_H_
#define TRACK_H_

#include <vector>

using std::vector;

class Track {
 public:
  Track();
  virtual ~Track();

  void processWpts();

  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);

  vector<double> getFrenet(double x, double y, double theta);
  vector<double> getXY(double s, double d);

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;

  // Frenet d unit normal vector (x,y)
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  int m_num_wpts;
  int m_num_lanes_available;

 private:

};

#endif /* TRACK_H_ */
