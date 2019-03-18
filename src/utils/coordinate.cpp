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
/*
Coordinate::Coordinate(double _roll, double _pitch, double _yaw, double _xgyro, double _ygyro, double _zgyro,
		double _xacc, double _yacc, double _zacc, double _x, double _y, double _z, coordMode mode):
		roll(_roll),pitch(_pitch),yaw(_yaw),
		x(_x),y(_y),z(_z),xgyro(_xgyro),ygyro(_ygyro),zgyro(_zgyro),xacc(_xacc),yacc(_yacc),zacc(_zacc),currCoordMode(mode)
{
    euler2quaternion(true);
}
*/
Coordinate::Coordinate(double _qw, double _qx, double _qy, double _qz, double _xgyro, double _ygyro, double _zgyro,
		double _xacc, double _yacc, double _zacc, double _x, double _y, double _z, coordMode mode):
		qw(_qw),qx(_qx),qy(_qy),qz(_qz),x(_x),y(_y),z(_z),xgyro(_xgyro),ygyro(_ygyro),zgyro(_zgyro),
		xacc(_xacc),yacc(_yacc),zacc(_zacc),currCoordMode(mode)
{
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
      //  currRotMode = euler;
        euler2quaternion(true);
    }
    else if (rotation.size() == 4)
    {
        qw = rotation[0];
        qx = rotation[1];
        qy = rotation[2];
        qz = rotation[3];
      //currRotMode = quaternion;

        euler2quaternion(false);
    }
    currCoordMode = mode;
}
void Coordinate::assign(sensor_msgs::Imu& imu_topic_msg)
{
	for (int i = 0; i < 9; i++)
	{
		imu_topic_msg.orientation_covariance[i] = 0;
		imu_topic_msg.angular_velocity_covariance[i] = 0;
		imu_topic_msg.linear_acceleration_covariance[i] = 0;
	}
	imu_topic_msg.orientation.w = qw;
	imu_topic_msg.orientation.x = qx;
	imu_topic_msg.orientation.y = qy;
	imu_topic_msg.orientation.z = qz;
	imu_topic_msg.angular_velocity.x = xgyro;
	imu_topic_msg.angular_velocity.y = ygyro;
	imu_topic_msg.angular_velocity.z = zgyro;
	imu_topic_msg.linear_acceleration.x = xacc;
	imu_topic_msg.linear_acceleration.y = yacc;
	imu_topic_msg.linear_acceleration.z = zacc;
}
void Coordinate::setGyro(double _xgyro, double _ygyro, double _zgyro)
{
	xgyro = _xgyro;
	ygyro = _ygyro;
	zgyro = _zgyro;
}
void Coordinate::setAcc(double _xacc, double _yacc, double _zacc)
{
	xacc = _xacc;
	yacc = _yacc;
	zacc = _zacc;
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
        if (mode == nedBody || currCoordMode == nedBody) // (currCoordMode)hlu-ENU <-> body-fixed hrd-NED, both directions are the same
        {
            y = -y;
            z = -z;
            ygyro = -ygyro;
            zgyro = -zgyro;
            yacc = -yacc;
            zacc = -zacc;
            pitch = -pitch;
            yaw = -yaw;
            qy = -qy;
            qz = -qz;
        }
        else if (mode == nedLocal || currCoordMode == nedLocal)	// (currCoordMode)ENU -> local NED
        {
            std::swap(x, y);
            z = -z;
            std::swap(xgyro, ygyro);
            zgyro = -zgyro;
            std::swap(xacc, yacc);
            zacc = -zacc;
            std::swap(roll, pitch);
            yaw = -yaw;
            std::swap(qx, qy);
            qz = -qz;
        }
    }
    currCoordMode = mode;
}
/*
void Coordinate::setRotMode(Coordinate::rotMode mode)
{
    if (currRotMode != mode)
    {
        currRotMode = mode;
        // no need to recalc
    }
}
*/
void Coordinate::setTranslation(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

void Coordinate::setTranslation(const std::vector<double>& translation)
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
    //currRotMode = euler;
    euler2quaternion(true);
}

void Coordinate::setRotation(double _qw, double _qx, double _qy, double _qz)
{
    qw = _qw;
    qx = _qx;
    qy = _qy;
    qz = _qz;
    //currRotMode = quaternion;
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

std::vector<double> Coordinate::getRotation(Coordinate::rotMode rot_mode) const
{
    if (rot_mode == euler)
        return std::vector<double>({roll, pitch, yaw});
    else if (rot_mode == quaternion)
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
