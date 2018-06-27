/*
 * Vehicle.h
 *
 *  Created on: Jun 22, 2018
 *      Author: deanliu
 */

#ifndef VEHICLE_H_
#define VEHICLE_H_

class Vehicle {
  public:
    Vehicle();
    Vehicle(double x, double y, double s, double d, double yaw, double spd);

    virtual ~Vehicle();


    struct Coord
    {
      double x;
      double y;
    };

//    double getX() const {return m_x;}
//    double getY() const {return m_y;}
//    double getS() const {return m_s;}
//    double getD() const {return m_d;}
//    double getYaw() const {return m_yaw;}
//    double getSpeed() const {return m_speed;}

//    void setX(double x) {m_x = x;}
//    void setY(double y) {m_y = y;}
//    void setS(double s) {m_s = s;}
//    void setD(double d) {m_d = d;}
//    void setYaw(double yaw) {m_yaw = yaw;}
//    void setSpeed(double speed) {m_speed = speed;}

    void setPose(double x, double y, double s, double d, double yaw, double spd);

//    void setPose(double x, double y, double s, double d, double vx, double vy);

//    Coord transformGlobal2Ego(Coord in);
//    Coord transformEgo2Global(Coord in);

  private:

    double m_x;     // global position x, [m]
    double m_y;     // global position y, [m]
    double m_s;     // frenet position s, [m]
    double m_d;     // frenet position d, [m]
    double m_yaw;   // global track angle, [rad] (+) rotate from x->y axis
    double m_speed; // speed, [m/s]
//    double m_vx;    // global velocity x-component, [m/s]
//    double m_vy;    // global velocity y-component, [m/s]

    double m_yaw_ds; // frenet track angle, [rad] (+)???
    double m_a;     // acceleration

};

#endif /* VEHICLE_H_ */
