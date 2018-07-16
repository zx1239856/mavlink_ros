/** 
 * Simple Coordinate Handler 
 * 
 * author: zx1239856@gmail.com
 * Under MIT License
 */
#include "coordinate.h"
#include <cassert>
#include <cmath>

Coordinate::Coordinate()
{
    euler2quaternion(true);
}

Coordinate::Coordinate(double _x, double _y, double _z, double _roll, double _pitch, double _yaw, coordMode mode)
{
    x = _x;
    y = _y;
    z = _z;
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    currCoordMode = mode;
    currRotMode = euler;
    euler2quaternion(true);
}

Coordinate::Coordinate(double _x, double _y, double _z, double _qw, double _qx, double _qy, double _qz, coordMode mode)
{
    x = _x;
    y = _y;
    z = _z;
    qw = _qw;
    qx = _qx;
    qy = _qy;
    qz = _qz;
    currCoordMode = mode;
    currRotMode = quaternion;
    euler2quaternion(false);
}

Coordinate::Coordinate(std::vector<double> translation, std::vector<double> rotation, coordMode mode)
{
    assert(translation.size() == 3 && (rotation.size() == 3 || rotation.size() == 4));
    x = translation[0];
    y = translation[1];
    z = translation[2];
    if (rotation.size() == 3)
    {
        roll = rotation[0];
        pitch = rotation[1];
        yaw = rotation[2];
        currRotMode = euler;
        euler2quaternion(true);
    }
    else if (rotation.size() == 4)
    {
        qw = rotation[0];
        qx = rotation[1];
        qy = rotation[2];
        qz = rotation[3];
        currRotMode = quaternion;
        euler2quaternion(false);
    }
    currCoordMode = mode;
}

void Coordinate::setCoordMode(Coordinate::coordMode mode)
{
    //body-fixed NED → ROS ENU: (x y z)→(x -y -z) or (w x y z)→(x -y -z w)
    //local NED → ROS ENU: (x y z)→(y x -z) or (w x y z)→(y x -z w)
    if (currCoordMode != mode)
    {
        // we cannot convert from local to body frame
        assert(!((currCoordMode == nedLocal && mode == nedBody) || (currCoordMode == nedBody && mode == nedLocal)));
        // do re-calc stuffs
        if (mode == nedBody || currCoordMode == nedBody) // ENU -> body-fixed NED
        {
            y = -y;
            z = -z;
            pitch = -pitch;
            yaw = -yaw;
            qy = -qy;
            qz = -qz;
        }
        else if (mode == nedLocal || currCoordMode == nedLocal)
        {
            std::swap(x, y);
            z = -z;
            std::swap(roll, pitch);
            yaw = -yaw;
            std::swap(qx, qy);
            qz = -qz;
        }
    }
    currCoordMode = mode;
}

void Coordinate::setRotMode(Coordinate::rotMode mode)
{
    if (currRotMode != mode)
    {
        currRotMode = mode;
        // no need to recalc
    }
}

void Coordinate::setTranslation(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

void Coordinate::setTranslation(std::vector<double> translation)
{
    x = translation[0];
    y = translation[1];
    z = translation[2];
}

void Coordinate::setRotation(double _roll, double _pitch, double _yaw)
{
    roll = _roll;
    pitch = _pitch;
    yaw = _yaw;
    currRotMode = euler;
    euler2quaternion(true);
}

void Coordinate::setRotation(double _qw, double _qx, double _qy, double _qz)
{
    qw = _qw;
    qx = _qx;
    qy = _qy;
    qz = _qz;
    currRotMode = quaternion;
    euler2quaternion(false);
}

void Coordinate::setRotation(std::vector<double> rotation)
{
    assert(rotation.size() == 3 || rotation.size() == 4);
    if (rotation.size() == 3)
    {
        roll = rotation[0];
        pitch = rotation[1];
        yaw = rotation[2];
        euler2quaternion(true);
    }
    else if (rotation.size() == 4)
    {
        qw = rotation[0];
        qx = rotation[1];
        qy = rotation[2];
        qz = rotation[3];
        euler2quaternion(false);
    }
}

std::vector<double> Coordinate::getTranslation() const
{
    return std::vector<double>({x, y, z});
}

std::vector<double> Coordinate::getRotation() const
{
    if (currRotMode == euler)
        return std::vector<double>({roll, pitch, yaw});
    else if (currRotMode == quaternion)
        return std::vector<double>({qw, qx, qy, qz});
}

// private helpers
void Coordinate::euler2quaternion(bool direction)
{
    if (direction) // euler -> quaternion
    {
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        qw = cy * cr * cp + sy * sr * sp;
        qx = cy * sr * cp - sy * cr * sp;
        qy = cy * cr * sp + sy * sr * cp;
        qz = sy * cr * cp - cy * sr * sp;
    }
    else // quaternion -> euler
    {
        roll = std::atan2(2 * (qw * qx + qy * qz), (1 - 2 * (qx * qx + qy * qy)));
        pitch = std::asin(2 * (qw * qy - qx * qz));
        yaw = std::atan2(2 * (qw * qz + qx * qy), (1 - 2 * (qz * qz + qy * qy)));
    }
}