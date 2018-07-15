/*
 * Vehicle.h
 *
 *  The Vehicle class is meant to represent a vehicle that is on the road. It holds
 *  the pose of itself via a Pose data structure, and it knows how to propagate that
 *  pose into the future.
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

    Pose propagatePose(const Pose& p_in, float t);

    vector<Pose> trajectoryPrediction(float time_horizon=2.0);

    Track& m_rTrack;  // reference to the track object for global/frenet conversion services

  protected:

    Pose propagatePoseInertial(const Pose& p_in, float t);
    Pose propagatePoseFrenet(const Pose& p_in, float t);

    Pose m_pose;

  private:

};

#endif /* VEHICLE_H_ */
