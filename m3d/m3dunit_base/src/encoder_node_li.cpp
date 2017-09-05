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
double encoderOffset;
void positionCallbac(const std_msgs::Float32ConstPtr& msg)
{
    m3d.setPosition(msg->data-encoderOffset,25,1);
}

void velocityCallback(const std_msgs::Float32ConstPtr& msg)
{
    m3d.setSpeed(msg->data);
}
int main(int argc, char** argv)
{


    ros::init(argc, argv, "m3d_driver");
    ros::NodeHandle node ("~");
    tf::TransformBroadcaster m3dRot;
    ros::Publisher angPub = node.advertise<std_msgs::Float32>("current_angle", 10);
    ros::Subscriber s1 =node.subscribe("velocity", 1, velocityCallback);
    ros::Subscriber s2 =node.subscribe("position", 1, positionCallbac);

    std::string ip;
    std::string serial_dev;

	std::string m3d_frame_id;
	std::string m3d_front_frame_id;
	std::string m3d_rot_frame_id;

    node.getParam("encoderOffset", encoderOffset);
    ROS_INFO("encoder offset %f", encoderOffset);
    encoderOffset = encoderOffset + M_PI;
	
	node.param<double>("encoderOffset", encoderOffset, 0);
    node.param<std::string>("ip", ip, "");
    node.param<std::string>("serial", serial_dev, "");


	node.param<std::string>("m3d_frame_id", m3d_frame_id, "m3d_link");
	node.param<std::string>("m3d_front_frame_id", m3d_front_frame_id, "m3d_front_laser_link");
	node.param<std::string>("m3d_rot_frame_id", m3d_rot_frame_id, "m3d_rot_laser_link");

    //ROS_INFO_ONCE("Try to connect to M3D unit @ %s",ip.c_str());

    if (ip.length()>0)
    {
        ROS_INFO_STREAM("connecting using TCP ip: " << ip);

        if (!m3d.connect_to_m3d_tcp(ip))
        {
            ROS_FATAL("Problem connecting to unit");
            return -1 ;
        }
    }
    else if (serial_dev.length()>0)
    {
        ROS_INFO_STREAM("connecting using SERIAL port: "<<serial_dev );

        if (!m3d.connect_to_m3d_serial(serial_dev))
        {
            ROS_FATAL("Problem connecting to unit");
            return -1 ;
        }
    }
    else
    {
        ROS_FATAL("no method specified");
        return -1;
    }

    ROS_INFO_ONCE("connected");
    tf::Transform front_laserLink;
    front_laserLink.setIdentity();
    front_laserLink.setOrigin(tf::Vector3(0.0285, 0 , 0.04));

	while (node.ok())
	{
		tf::Transform transform;
        	transform.setOrigin(tf::Vector3(-0.0835, 0, 0.1835));
		tf::Quaternion q;
		//int ang = e.readActualAng();
		if (true)
		{
			ROS_INFO_ONCE("Started to get data");
            float ang = m3d.getAngle()-encoderOffset;
            ROS_DEBUG("angle %f",ang);
            q.setRPY(0,-M_PI_2,ang);
            q.normalize();
            std_msgs::Float32 f;
            f.data= ang;
            angPub.publish(f);
            transform.setRotation(q);
            m3dRot.sendTransform(tf::StampedTransform(transform, ros::Time::now(), m3d_frame_id.c_str(), m3d_rot_frame_id.c_str()));
            m3dRot.sendTransform(tf::StampedTransform(front_laserLink, ros::Time::now(), m3d_frame_id.c_str(), m3d_front_frame_id));

            ros::spinOnce();
        }
	}
	m3d.setSpeed(0);
}
