# mavlink_ros(Deprecated)
No support for newer PX4 Firmware

## Changelogs
+ Version 1.1.0  Add UDP support  
+ Version 1.0.0  Change project name to mavlink_ros with brand new code architecture, UDP support not implemented for now
+ Version 0.1.0  Support UDP forward
+ Version 0.0.0  Initial Version 

## Description
A simple tool for communication between PX4 and ROS. 

## ROS Topics Used
+ /imu  
Publish imu data from PX4 flight controller. This can be used later in SLAM applications.
+ /mavReadRaw  
Publish mavlink rawdata read from serial. This can be utilized by other modules, i.e. UDP forwarding module. 
+ /mavWriteRaw  
Everything published to this topic gets written to serial in the order of incoming time.

## Modules
+ serialpub  
ROS node to publish data from serial interface  
Usage: 
```serialpub [devName]```
+ udpnode  
Forward PX4 data to QGroundControl through UDP protocol
