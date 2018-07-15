/*
 * Pose.h
 *
 *  This include file contains a Pose struct which holds all the state data of a
 *  vehicle in an instance in time (ie: pose). It also contains a FSM_state enum
 *  that defines the Finite State Machine (FSM) states of the vehicle's path planner.
 *
 *  This file is meant to be included by any classes that will handle data pertaining
 *  to a vehicle's pose or FSM states so that they have the definition of these complex
 *  data types. Thus this file acts as part of the interface (API).
 *
 *  Created on: Jun 26, 2018
 *      Author: deanliu
 */

#ifndef POSE_H_
#define POSE_H_

#include <string>
#include <iostream>
#include <map>

using std::string;
using std::ostream;
using std::map;

enum FSM_state
{
  kFSM_None = 0,
  kFSM_Start, // 1
  kFSM_CS,    // 2
  kFSM_KL,    // 3
  kFSM_PLCL,  // 4
  kFSM_PLCR,  // 5
  kFSM_LCL,   // 6
  kFSM_LCR    // 7
};

/* overload << operator so std::cout prints enum name as string, instead of int value
 * https://stackoverflow.com/questions/3342726/c-print-out-enum-value-as-text
 * This is currently getting "duplicate symbols" linking error.
 */
//ostream& operator << (ostream& out, const FSM_state value)
//{
//  static map<FSM_state, string> strings;
//  if (strings.size() == 0)
//  {
//#define INSERT_ELEMENT(p) strings[p] = #p
//        INSERT_ELEMENT(kFSM_None);
//        INSERT_ELEMENT(kFSM_Start);
//        INSERT_ELEMENT(kFSM_CS);
//        INSERT_ELEMENT(kFSM_KL);
//        INSERT_ELEMENT(kFSM_PLCL);
//        INSERT_ELEMENT(kFSM_PLCR);
//        INSERT_ELEMENT(kFSM_LCL);
//        INSERT_ELEMENT(kFSM_LCR);
//#undef  INSERT_ELEMENT
//  }
//  return out << strings[value];
//}

struct Pose
{
  double x;  // [m] Cartesian coord
  double y;  // [m] Cartesian coord
  double s;  // [m] Frenet coord
  double d;  // [m] Frenet coord
  double yaw; // [rad] Cartesian coord, (+) from X-axis to Y-axis
  double spd; // [m/s]
  int    lane;// 0 is far left, each lane on right +1
  FSM_state state = kFSM_None; // change this to enum later
};

#endif /* POSE_H_ */
