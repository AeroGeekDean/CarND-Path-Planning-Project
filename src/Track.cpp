/*
 * Track.cpp
 *
 *  Created on: Jun 28, 2018
 *      Author: deanliu
 */

#include "Track.h"
#include "UtilFunctions.h"

using std::min;

Track::Track()
{
  map_waypoints_x.clear();
  map_waypoints_y.clear();
  map_waypoints_s.clear();
  map_waypoints_dx.clear();
  map_waypoints_dy.clear();
}

Track::~Track()
{
}

void Track::processWpts()
{
  m_num_wpts = map_waypoints_x.size();
}

/*************************
 * ClosestWaypoint()
 * Finds the closest WPT
 ************************/
int Track::ClosestWaypoint(double x, double y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i=0; i<m_num_wpts; i++)
  {
    double map_x = map_waypoints_x[i];
    double map_y = map_waypoints_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

/************************
 * NextWaypoint()
 * Finds the next WPT
 * (based on car orientation ONLY!!)
 ************************/
int Track::NextWaypoint(double x, double y, double theta)
{
  int closestWaypoint = ClosestWaypoint(x,y);

  double map_x = map_waypoints_x[closestWaypoint];
  double map_y = map_waypoints_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading); // bearing to WPT off bow
  angle = min((2*pi()-angle), angle); // if > 180deg, then measure from the other direction

  if(angle > pi()/2) { // if bearing-off-bow is beyond 90deg, closest WPT is behind, then grab next WPT
    closestWaypoint++;
    closestWaypoint = closestWaypoint % m_num_wpts; // wrap to protect against OutOfBound index
  }
  return closestWaypoint;
}

/***********************
 * getFrenet()
 * Transform from map x,y coordinates to Frenet s,d coordinates
************************/
vector<double> Track::getFrenet(double x, double y, double theta)
{
  int next_wp = NextWaypoint(x,y, theta);
  int prev_wp = next_wp-1;

  // detect for wrapping around the WPTS (since it's a loop)
  if(next_wp == 0)  {
    prev_wp = m_num_wpts-1;
  }

  double n_x = map_waypoints_x[next_wp] - map_waypoints_x[prev_wp]; // vector 'n' from prev_wp to next_wp
  double n_y = map_waypoints_y[next_wp] - map_waypoints_y[prev_wp];

  double x_x = x - map_waypoints_x[prev_wp];  // vector 'X' from prev_wp to vehicle location
  double x_y = y - map_waypoints_y[prev_wp];

  /* Use vector cross-product to calc frenet_d value (Sarrus's Rule)
     Ref: https://en.wikipedia.org/wiki/Cross_product#Matrix_notation
     The along-leg-path vector needs to be an unit-vector (length=1) for this to work
  */
  double leg_length = distance(map_waypoints_x[next_wp], map_waypoints_y[next_wp],
                               map_waypoints_x[prev_wp], map_waypoints_y[prev_wp]);

  n_x = n_x / leg_length; // make it an unit-vector...
  n_y = n_y / leg_length;

  // Sarrus's Rule for the k-axis (vector 'n' cross-product vector 'X')
  double frenet_d = x_x*n_y - x_y*n_x;  // positive (+) to RIGHT of centerline path

  // calculate S value as that of prev_wp + S along this leg length
  double frenet_s = map_waypoints_s[prev_wp]
                  + (x_x*n_x + x_y*n_y); // 'X' dot-product with unit-vector 'n'

//  for(int i=0; i<prev_wp; i++)  {
//    // now add all prior leg lengths
//    frenet_s += distance(map_waypoints_x[i], map_waypoints_y[i],
//                         map_waypoints_x[i+1], map_waypoints_y[i+1]);
//  }

  return {frenet_s,frenet_d};
}

/***********************
 * getXY()
 * Transform from Frenet s,d coordinates to map x,y
************************/
vector<double> Track::getXY(double s, double d)
{
  int prev_wp = -1;

  s = s_wrap(s); // make sure S is within max_s bounds

  while( (s >= map_waypoints_s[prev_wp+1]) && // find the correct prev_wp
         (prev_wp < (m_num_wpts-1)) ) // s is after the last wp
  {
    prev_wp++;
  }

  int next_wp = (prev_wp+1)%m_num_wpts;

  double heading = atan2((map_waypoints_y[next_wp]-map_waypoints_y[prev_wp]),
                         (map_waypoints_x[next_wp]-map_waypoints_x[prev_wp]));

  // the x,y,s along the segment
  double seg_s = (s - map_waypoints_s[prev_wp]);
  double seg_x = map_waypoints_x[prev_wp] + seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp] + seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y,heading};
}
