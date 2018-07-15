/*
 * Track.h
 *
 *  The track class holds knowledge about the closed-circuit track that the car
 *  will be driven on. It contains waypoints (wpts) that define the centerline
 *  of the track, with 3 lanes on each sides of the centerline. The wapoints data
 *  contains (x, y) global coordinates for each waypoint, as well as the Frenet-S
 *  distance for each waypoints. Additionally, a (dx, dy) unit vector in global
 *  coordinate is also provided to define the direction of the Frenet-D.
 *
 *  Methods to convert between global (x,y) and Frenet (s, d) are provided, as
 *  well as methods to find both the closest waypoint and next waypoint (based
 *  on orientation).
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
  float m_max_s;

 private:

};

#endif /* TRACK_H_ */
