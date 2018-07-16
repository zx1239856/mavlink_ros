/** 
 * Simple Coordinate Handler 
 * 
 * author: zx1239856@gmail.com
 * Under MIT License
 */
#pragma once

#include <vector>
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
        enu,
        nedLocal,
        nedBody
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
    rotMode currRotMode = euler;
    double x = 0, y = 0, z = 0;            // cartesian coord
    double roll = 0, pitch = 0, yaw = 0;   // euler angle
    double qw = 0, qx = 0, qy = 0, qz = 0; // quaternion
    // helper functions
    void euler2quaternion(bool direction); // true for euler->quaternion, false for quaternion -> euler

  public:
    Coordinate();
    Coordinate(double _x, double _y, double _z, double _roll, double _pitch, double _yaw, coordMode _currCoordMode = enu);
    Coordinate(double _x, double _y, double _z, double _qw, double _qx, double _qy, double _qz, coordMode _currCoordMode = enu);
    Coordinate(std::vector<double> translation, std::vector<double> rotation, coordMode _currCoordMode = enu);
    void setCoordMode(Coordinate::coordMode);
    void setRotMode(Coordinate::rotMode);
    void setTranslation(double _x, double _y, double _z);
    void setTranslation(std::vector<double> translation);
    void setRotation(double _roll, double _pitch, double _yaw);
    void setRotation(double _qw, double _qx, double _qy, double _qz);
    void setRotation(std::vector<double> rotation);
    std::vector<double> getTranslation() const;
    std::vector<double> getRotation() const;
};