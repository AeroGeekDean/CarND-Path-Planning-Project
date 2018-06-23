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
    virtual ~Vehicle();


    struct Coord
    {
      double x;
      double y;
    };

    double getX() const {return m_x;}
    double getY() const {return m_y;}
    double getS() const {return m_s;}
    double getD() const {return m_d;}
    double getYaw() const {return m_yaw;}
    double getSpeed() const {return m_speed;}

//    void setX(double x) {m_x = x;}
//    void setY(double y) {m_y = y;}
//    void setS(double s) {m_s = s;}
//    void setD(double d) {m_d = d;}
//    void setYaw(double yaw) {m_yaw = yaw;}
//    void setSpeed(double speed) {m_speed = speed;}

    void setPose(double x, double y, double s, double d, double yaw, double spd);

    Coord transformGlobal2Ego(Coord in);
    Coord transformEgo2Global(Coord in);

  private:
    double m_x;     // [m]
    double m_y;     // [m]
    double m_s;     // [m]
    double m_d;     // [m]
    double m_yaw;   // [rad]
    double m_speed; // [mph]

};

#endif /* VEHICLE_H_ */
