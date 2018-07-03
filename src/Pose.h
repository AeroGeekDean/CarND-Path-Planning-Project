#ifndef POSE_H_
#define POSE_H_

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
};

#endif /* POSE_H_ */
