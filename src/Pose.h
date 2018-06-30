#ifndef POSE_H_
#define POSE_H_

struct Pose
{
  double x;  // [m]
  double y;  // [m]
  double s;  // [m]
  double d;  // [m]
  double vx;  // [m/s]
  double vy;  // [m/s]
  double yaw; // [rad]
  double spd; // [m/s]
};

#endif /* POSE_H_ */
