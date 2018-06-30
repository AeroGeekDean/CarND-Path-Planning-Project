/*
 * Vehicle.h
 *
 *  Created on: Jun 22, 2018
 *      Author: deanliu
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

#include "Track.h"
#include "Pose.h"
#include <vector>

using std::vector;

class Vehicle {
  public:

    Vehicle(Track& trk);

    virtual ~Vehicle();

    void updatePose(const Pose& p);
    const Pose& getPose() const { return m_pose; }

    Pose propagatePose(float t);
    Pose propagatePoseInertial(float t);
    Pose propagatePoseFrenet(float t);

    vector<Pose> trajectoryPrediction(float time_horizon=2.0, float dt=0.5);

    int getLane() const { return m_lane; }

    Track& m_rTrack;  // reference to the track object for global/frenet conversion services

  protected:

    Pose m_pose;

    int m_lane; // starting with far left lane = 0
    int lanes_available = 3;

//    double m_vx;    // global velocity x-component, [m/s]
//    double m_vy;    // global velocity y-component, [m/s]

    double m_yaw_ds; // frenet track angle, [rad] (+)???
    double m_a;     // acceleration

  private:

};

#endif /* VEHICLE_H_ */
