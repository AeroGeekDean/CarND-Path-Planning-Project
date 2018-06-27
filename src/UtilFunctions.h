/*
 * UtilFunctions.h
 *
 *  Created on: Nov 1, 2017
 *      Author: deanliu
 */

#ifndef UTILFUNCTIONS_H_
#define UTILFUNCTIONS_H_

#include <math.h>

using std::vector;
using std::min;

const double mph2ms = 0.45;
const double ms2mph = 1/mph2ms;

// For converting back and forth between radian and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

/************************
 * magnitude()
 ***********************/
inline double magnitude(double x, double y) { return sqrt(x*x+y*y); }

/************************
 * distance()
 ***********************/
inline double distance(double x1, double y1, double x2, double y2)
{
  return magnitude((x2-x1), (y2-y1));
//  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

/*************************
 * ClosestWaypoint()
 * Finds the closest WPT
 ************************/
inline int ClosestWaypoint(double x,
                    double y,
                    const vector<double> &maps_x,
                    const vector<double> &maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
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
inline int NextWaypoint(double x,
                 double y,
                 double theta,
                 const vector<double> &maps_x,
                 const vector<double> &maps_y)
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading); // bearing to WPT off bow
  angle = min((2*pi()-angle), angle); // if > 180deg, then measure from the other direction

  if(angle > pi()/2) { // if bearing-off-bow is beyond 90deg, closest WPT is behind, then grab next WPT
    closestWaypoint++;
    closestWaypoint = closestWaypoint % maps_x.size(); // wrap to protect against OutOfBound index
  }
  return closestWaypoint;
}

/***********************
 * getFrenet()
 * Transform from map x,y coordinates to Frenet s,d coordinates
************************/
inline vector<double> getFrenet(double x,
                         double y,
                         double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
  int prev_wp = next_wp-1;

  // detect for wrapping around the WPTS (since it's a loop)
  if(next_wp == 0)  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp]; // vector 'n' from prev_wp to next_wp
  double n_y = maps_y[next_wp] - maps_y[prev_wp];

  double x_x = x - maps_x[prev_wp];  // vector 'X' from prev_wp to vehicle location
  double x_y = y - maps_y[prev_wp];

//-------------------------------------------------------------------------------
//-- this is SOooo kludgy! Where did the magic point (1000,2000) came from?
//--

  if (0) // skip this whole segment since it's terrible algorithm
  {
  // find the projection of x onto n
  double proj_norm = (x_x*n_x + x_y*n_y) / (n_x*n_x + n_y*n_y); // normalized dot-product
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);
  if(centerToPos <= centerToRef)  {
    frenet_d *= -1;
    }
  }
//---------------------- end of kludge -----------------------------------------


  /* Use vector cross-product to calc frenet_d value (Sarrus's Rule)
     Ref: https://en.wikipedia.org/wiki/Cross_product#Matrix_notation
     The along-leg-path vector needs to be an unit-vector (length=1) for this to work
  */
  double leg_length = distance(maps_x[next_wp], maps_y[next_wp],
                               maps_x[prev_wp], maps_y[prev_wp]);

  n_x = n_x / leg_length; // make it an unit-vector...
  n_y = n_y / leg_length;

  // Sarrus's Rule for the k-axis (vector 'n' cross-product vector 'X')
  double frenet_d = x_x*n_y - x_y*n_x;  // positive (+) to RIGHT of centerline path

  // calculate s value along this leg length
  double frenet_s = x_x*n_x + x_y*n_y; // 'X' dot-product with unit-vector 'n'

  for(int i = 0; i < prev_wp; i++)  {
    // now add all prior leg lengths
    frenet_s += distance(maps_x[i], maps_y[i],
                         maps_x[i+1], maps_y[i+1]);
  }

  return {frenet_s,frenet_d};
}

/***********************
 * getXY()
 * Transform from Frenet s,d coordinates to map x,y
************************/
inline vector<double> getXY(double s,
                     double d,
                     const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y)
{
  int prev_wp = -1;

//  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
  while(s >= maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))  {  // protect for case of s=0.0
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]), (maps_x[wp2]-maps_x[prev_wp]));

  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}


/***********************
 * global2local()
************************/
inline vector<double> global2local(double x_in, double y_in, double x_ref, double y_ref, double del_theta)
{
 vector<double> out;
 double dx = x_in-x_ref; // translation
 double dy = y_in-y_ref;
 out.push_back( dx*cos(del_theta) - dy*sin(del_theta) ); // rotation
 out.push_back( dx*sin(del_theta) + dy*cos(del_theta) );
 return out;
}

/***********************
 * local2global()
************************/
inline vector<double> local2global(double x_in, double y_in, double x_ref, double y_ref, double del_theta)
{
  vector<double> out;
  out.push_back( (x_in*cos(del_theta) - y_in*sin(del_theta)) + x_ref ); // first rotate then translate
  out.push_back( (x_in*sin(del_theta) + y_in*cos(del_theta)) + y_ref );
  return out;
}


/***********************
************************/
#endif /* UTILFUNCTIONS_H_ */
