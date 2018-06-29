/*
 * Vehicle.h
 *
 *  Created on: Jun 22, 2018
 *      Author: deanliu
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "Track.h"
#include <vector>

using std::vector;

class Vehicle {
  public:
    struct Pose
    {
      double x;
      double y;
      double s;
      double d;
      double vx;
      double vy;
      double yaw;
      double spd;
    };

    Vehicle(Track& trk);
    Vehicle(Track& trk, Pose p);

    virtual ~Vehicle();

    void setPose(Pose p);
    Pose propagatePose(float t);

    vector<Pose> trajectoryPrediction(float time_horizon=2.0, float dt=0.5);

  protected:

    Pose m_pose;

    int lane; // starting with far left lane = 0
    int lanes_available = 3;

//    double m_vx;    // global velocity x-component, [m/s]
//    double m_vy;    // global velocity y-component, [m/s]

    double m_yaw_ds; // frenet track angle, [rad] (+)???
    double m_a;     // acceleration

    Track& track;  // reference to the track object for global/frenet conversion services

  private:

};

#endif /* VEHICLE_H_ */
