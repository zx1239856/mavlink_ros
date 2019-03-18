/** 
 * Simple Coordinate Handler 
 * 
 * author: zx1239856@gmail.com
 * Under MIT License
 */
#pragma once

#include <vector>
#include <Eigen/Dense>
#include "sensor_msgs/Imu.h"
/** 
* ENU in ROS => NED in PX4 coordinate frame conversion guidance 
* 
* (x,y,z) => (x,-y,-z)
* (w,x,y,z) => (x,-y,-z,w) 
* (roll,pitch,yaw) => (roll,-pitch,-yaw)
*/

class Coordinate
{
  public:
    enum coordMode
    {
        enu,		// ros
        nedLocal,
        nedBody		// px4
    };
    // coordinate mode
    enum rotMode
    {
        euler,
        quaternion
    };
    // mode to represent rotation, either euler angle or quaternion
  private:
    coordMode currCoordMode = enu;
  //  rotMode currRotMode = euler;
    double x = 0, y = 0, z = 0;            	// cartesian coord
    double xgyro = 0, ygyro = 0, zgyro = 0;	// cartesian coord
    double xacc = 0, yacc = 0, zacc = 0;	// cartesian coord
    double roll = 0, pitch = 0, yaw = 0;   	// euler angle
    double qw = 0, qx = 0, qy = 0, qz = 0; 	// quaternion
    // helper functions
    void euler2quaternion(bool direction); // true for euler->quaternion, false for quaternion -> euler
  public:
    Coordinate();
 //   Coordinate(double _roll, double _pitch, double _yaw, double _xgyro = 0, double _ygyro = 0, double _zgyro = 0,
  //  		double _xacc = 0, double _yacc = 0, double _zacc = 0, double _x = 0, double _y = 0, double _z = 0, coordMode mode = enu);
    Coordinate(double _qw, double _qx, double _qy, double _qz, double _xgyro = 0, double _ygyro = 0, double _zgyro = 0,
    		double _xacc = 0, double _yacc = 0, double _zacc = 0, double _x = 0, double _y = 0, double _z = 0, coordMode mode = enu);
    Coordinate(std::vector<double> translation, std::vector<double> rotation, coordMode _currCoordMode = enu);
    void setCoordMode(Coordinate::coordMode);
  //  void setRotMode(Coordinate::rotMode);
    void assign(sensor_msgs::Imu& imu_topic_msg);
    void setTranslation(double _x, double _y, double _z);
    void setGyro(double _roll, double _pitch, double _yaw);
    void setAcc(double _xacc, double _yacc, double _zacc);
    void setTranslation(const std::vector<double>& translation);
    void setRotation(double _roll, double _pitch, double _yaw);
    void setRotation(double _qw, double _qx, double _qy, double _qz);
    void setRotation(std::vector<double> rotation);
    std::vector<double> getTranslation() const;
    std::vector<double> getRotation(Coordinate::rotMode rot_mode) const;
};
