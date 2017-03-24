#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "lms_mini_lib.hpp"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
	
int main(int argc, char **argv)
{

    ros::init(argc, argv, "lms_mini_driver");
    ros::NodeHandle n("~");
    ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laserScan",1);
    lms_socket LMS;

    std::string ip;
    double startAngle;
    bool invert;
    std::string frame_id;


	n.param<std::string>("ip", ip, "192.168.0.201");
	n.param<double>("sa", startAngle, -95.0);
	n.param<bool>("inv", invert, 0);
	n.param<std::string>("frame_id", frame_id,"laser");

    ROS_INFO("inverted: %d",invert);
    ROS_INFO("Start angle: %f",startAngle);
    ROS_INFO("Will connect to laser '%s' and publish measurment with frame '%s'",ip.c_str() ,frame_id.c_str());

    double  rate =350;

	bool make_invert = false;
    if (invert)
	{
		make_invert = true;
		ROS_INFO ("SCANS WILL BE INVERTED");
	}
    LMS.connectToLaser(ip);

    LMS.requestContinousScan();

    ros::Rate r(rate);

    bool isFirst =true;
   while (n.ok())
    {

        bool isMeasurmet = false;
        LMS.readData(isMeasurmet);

	
        if (isMeasurmet)
        {
            if(isFirst)
            {
                ROS_INFO("Scanning started with serial: %d, device no: %d ", LMS.currentMessage._serialNo, LMS.currentMessage._deviceNo);
                isFirst =false;
            }

            sensor_msgs::LaserScan scan;
            scan.header.frame_id =frame_id;
            if (LMS.currentMessage.echoes.size()>0)
            {
                lms_channel* dist1 = &(LMS.currentMessage.echoes[0]);
                lms_channel* intens = NULL;
                if (LMS.currentMessage.rssis.size()>0)intens= &(LMS.currentMessage.rssis[0]);
                scan.header.stamp = ros::Time::now();
                scan.angle_increment = dist1->angStepWidth*M_PI/180;
                scan.angle_min = degreesToRadians(startAngle);
                scan.angle_max = degreesToRadians(-startAngle);
                scan.range_min =0.0;
                scan.range_max =100.0;
                scan.ranges.resize(dist1->data.size());

                if (intens)
                {
                    scan.intensities.resize(intens->data.size());
                }
                double scale = 0.001*dist1->scallingFactor;

                if (!make_invert)
                {
                        for (int i=0; i< dist1->data.size(); i++)
                        {
                            scan.ranges[i] = scale*dist1->data[i];

                        }
                }
                else
                {
                        for (int i=0; i< dist1->data.size(); i++)
                        {
                            scan.ranges[dist1->data.size()-1-i] = scale*dist1->data[i];
                        }
                }

                if (intens)
                {
                    if (!make_invert)
                    {
                            for (int i=0; i< intens->data.size(); i++)
                            {
                                scan.intensities[i] = intens->data[i];

                            }
                    }
                    else
                    {
                            for (int i=0; i< intens->data.size(); i++)
                            {
                                scan.intensities[dist1->data.size()-1-i] = intens->data[i];
                            }
                    }
                }

                laser_pub.publish(scan);
                ros::spinOnce();
                r.sleep();
                }
          }

    }

    ROS_INFO("closing connection");
    LMS.disconnet();
    return 0;
}
