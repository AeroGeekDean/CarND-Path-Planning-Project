#ifndef POSE_H_
#define POSE_H_

#include <string>
using std::string;

struct Pose
{
  double x;  // [m] Cartesian coord
  double y;  // [m] Cartesian coord
  double s;  // [m] Frenet coord
  double d;  // [m] Frenet coord
  double vx;  // [m/s] Cartesian coord
  double vy;  // [m/s] Cartesian coord
  double yaw; // [rad] Cartesian coord, (+) from X-axis to Y-axis
  double spd; // [m/s]
  int    lane;// 0 is far left, each lane on right +1
  string state; // change this to enum later
};

#endif /* POSE_H_ */
