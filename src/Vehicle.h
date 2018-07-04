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

    Pose propagatePose(const Pose& p_init, float t);

    vector<Pose> trajectoryPrediction(float time_horizon=2.0);

    int getLane() const { return m_lane; }

    Track& m_rTrack;  // reference to the track object for global/frenet conversion services

  protected:

    Pose propagatePoseInertial(const Pose& p_init, float t);
    Pose propagatePoseFrenet(const Pose& p_init, float t);

    Pose m_pose;

    int m_lane; // starting with far left lane = 0
    int lanes_available = 3;

    double m_yaw_ds; // frenet track angle, [rad] (+)???
    double m_a;     // acceleration

  private:

};

#endif /* VEHICLE_H_ */
