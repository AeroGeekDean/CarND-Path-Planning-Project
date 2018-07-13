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

const double max_s = 6945.554; // [m], length of track in Frenet coord

// For converting back and forth between radian and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

inline double magnitude(double x, double y) { return sqrt(x*x+y*y); }
inline double distance(double x1, double y1, double x2, double y2) { return magnitude((x2-x1), (y2-y1)); }

/***********************
 * global2local()
 ***********************/
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
 ***********************/
inline vector<double> local2global(double x_in, double y_in, double x_ref, double y_ref, double del_theta)
{
  vector<double> out;
  out.push_back( (x_in*cos(del_theta) - y_in*sin(del_theta)) + x_ref ); // first rotate then translate
  out.push_back( (x_in*sin(del_theta) + y_in*cos(del_theta)) + y_ref );
  return out;
}

/***********************
 * s_wrap() Wraps S to range [0, max_s]
 ***********************/
inline double s_wrap(const double& s_in)
{
  double s_out = s_in;
  while (s_out > max_s) s_out -= max_s;
  while (s_out <=  0.0) s_out += max_s;
  return s_out;
}


/***********************
 * s_wrap2() Wraps S to range [-max_s/2, max_s/2]
 ***********************/
inline double s_wrap2(const double& s_in)
{
  double s_out = s_in;
  while (s_out >   max_s/2) s_out -= max_s;
  while (s_out <= -max_s/2) s_out += max_s;
  return s_out;
}


/***********************
 ***********************/
#endif /* UTILFUNCTIONS_H_ */
