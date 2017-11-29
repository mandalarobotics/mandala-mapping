// This is low-integrity driver.
// This driver will publish only transform from 3dunit_link to 3d_unit_rot_laser
// Another node will agragate everything to pointclound.
// High integirty driver containts build-in cpp code laser driver. This Driver can be used only with SICK range finders.
// High integrate driver is meant for porting driver from ROS to another systems.

#include <iostream>
#include "driverLib.hpp"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

driver_m3d m3d;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "m3d_driver");
  ros::NodeHandle node ("~");
  tf::TransformBroadcaster m3dRot;
  ros::Publisher angPub = node.advertise<std_msgs::Float32>("current_angle", 10);

  std::string ip;
  std::string serial_dev;

  int offset;
  node.param<std::string>("ip", ip, "");
  node.param<std::string>("serial", serial_dev, "");

  node.param<int>("offset", offset, 0);
  //ROS_INFO_ONCE("Try to connect to M3D unit @ %s",ip.c_str());

  if (ip.length()>0)
  {
    ROS_INFO_STREAM("Connecting to ip: " << ip);

    if (!m3d.connect_to_m3d_tcp(ip))
    {
      ROS_FATAL(("Can't connect to IP:"+ip).c_str());
      return -1 ;
    }
  }
  else if (serial_dev.length()>0)
  {
    ROS_INFO_STREAM("Connecting to SERIAL port: "<<serial_dev );

    if (!m3d.connect_to_m3d_serial(serial_dev))
    {
      ROS_FATAL(("Can't connect to serial:"+serial_dev).c_str());
      return -1 ;
    }
  }
  else
  {
    ROS_FATAL("no method specified");
    return -1;
  }

  ROS_INFO_ONCE("connected");

  int oldOffset;
  m3d.getParam(0x37B3, 0x00, oldOffset);
  ROS_INFO_STREAM("Old offset :  "<<oldOffset );

  //37B3.00h Homing - offset
  m3d.writeParam(0x37B3, 0x00, offset);
  ROS_INFO_STREAM("New Offset :  "<< offset);

  //save all params
  m3d.writeParam(0x1010, 0x01, 0x65766173);

}
