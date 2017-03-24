#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


typedef pcl::PointXYZ PointType;

class pointCloudAggregator
{


public:
	pointCloudAggregator()
	{
		pc_agg = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
		angularDistance = 1.1*M_PI;
		firstScan = false;
	}
	void addPoints (PointType &p, tf::Transform &transform)
	{
		if (!creatingPointCloud) return;
		tf::Vector3 p0(p.x,p.y,p.z);
		tf::Vector3 p1;
		p1 = transform*p0;
		PointType pp = p;
		pp.x = p1[0];
		pp.y = p1[1];
		pp.z = p1[2];
		pc_agg->push_back(pp);

		// compute addition of traveled angular distance
		tf::Quaternion q = transform.getRotation();
		if (firstScan)
		{
			begin = q;
			firstScan = false;
			actual = q;
		}
		else
		{
			double d_distance = q.angleShortestPath(actual);
			if (!isnan(d_distance))currentAngularDistance=currentAngularDistance + d_distance ;
			actual = q;
		}
	}

	bool createPointCloud()
	{
		creatingPointCloud = true;
		firstScan = true;
	}
	bool isPointcloudReady()
	{

		if ( currentAngularDistance > angularDistance)
		{
			return true;
		}
		return false;
	}
	bool getCreatingPointcloud()
	{
		return creatingPointCloud;
	}
	void clearPointCloud()
	{
		pc_agg->clear();
		currentAngularDistance = 0;
		firstScan= false;
		creatingPointCloud = false;
	}
	pcl::PointCloud<PointType>::Ptr getPointCloud()
	{
		return pc_agg->makeShared();
	}
	double getProgress()
	{
		if (!creatingPointCloud) return -1;
		return 0.1*(floor(currentAngularDistance*1000.0/angularDistance));
		return 0;
	}

private:
	pcl::PointCloud<PointType>::Ptr pc_agg;
	tf::Quaternion begin;
	tf::Quaternion actual;
	double currentAngularDistance;
	double angularDistance;
	bool creatingPointCloud;
	bool firstScan;
};


class aggregator_node
{
public:
	aggregator_node ():
	n("~"),tfListener(tfBuffer)
	{
		n.param<std::string>("pointCloudFrame", rootTransform, "m3d_test/m3d_link");
		n.param<std::string>("rotLaserScan", rotLaserScan, "/m3d_test/rot_scan");
		n.param<std::string>("rotLaserPointCloud", rotLaserPointCloud, "");

		cloud_pub = n.advertise<sensor_msgs::PointCloud2>("cloud", 1);
		progress_pub = n.advertise<std_msgs::Float32>("progress", 1);
		done_pub =  n.advertise<std_msgs::Bool>("done", 1);

		rqt_sub        = n.subscribe("request", 1, &aggregator_node::requestCallback, this);
		if (rotLaserScan.size()!=0)
			laserScan_sub  = n.subscribe(rotLaserScan, 1, &aggregator_node::rotLaserScanCallback, this);
		if (rotLaserPointCloud.size()!=0)
			pointCloud_sub = n.subscribe(rotLaserPointCloud, 1, &aggregator_node::rotLaserPointCloudCallback, this);
		mPointCloudAggregator.createPointCloud();
		ros::spin();
	}

	void publishPointcloud()
	{
		std_msgs::Float32 progress;
		progress.data = mPointCloudAggregator.getProgress();
		progress_pub.publish(progress);

		if (mPointCloudAggregator.isPointcloudReady())
		{
			pcl::PointCloud<PointType> pointcloud;
			pointcloud = *(mPointCloudAggregator.getPointCloud());
			pcl::PCLPointCloud2 pcl_pc2;
			pcl::toPCLPointCloud2(pointcloud,pcl_pc2);
			sensor_msgs::PointCloud2 cloud;
			pcl_conversions::fromPCL(pcl_pc2,cloud);

			cloud.header.frame_id=rootTransform;
			cloud.header.stamp=ros::Time::now();

			std_msgs::Bool doneMsg;
			doneMsg.data = true;
			done_pub.publish(doneMsg);
			cloud_pub.publish(cloud);

			ROS_INFO("Publish pointcloud");
			mPointCloudAggregator.clearPointCloud();
		}
		else
		{
			if (mPointCloudAggregator.getCreatingPointcloud())
			{
				std_msgs::Bool doneMsg;
				doneMsg.data = false;
				done_pub.publish(doneMsg);
			}
		}
	}
	void requestCallback(const std_msgs::BoolPtr& req)
	{
		if (req->data != true) return;
		mPointCloudAggregator.clearPointCloud();
		mPointCloudAggregator.createPointCloud();
	}

	void rotLaserPointCloudCallback(const sensor_msgs::PointCloud2Ptr& scan )
	{
		ROS_DEBUG("Recieved scan, frame : %s", scan->header.frame_id.c_str());
		geometry_msgs::TransformStamped transform;
	    try{
	    	tfBuffer.canTransform(rootTransform, scan->header.frame_id, scan->header.stamp, ros::Duration(0.2));
	    	transform = tfBuffer.lookupTransform(rootTransform, scan->header.frame_id, scan->header.stamp);
	    }
	    catch (tf2::TransformException &ex) {
	      ROS_WARN("%s",ex.what());
	    }

	    pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(*scan,pcl_pc2);
		pcl::PointCloud<PointType>::Ptr temp_cloud(new pcl::PointCloud<PointType>);
		pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
		tf::Transform tf_transform;
		tf::transformMsgToTF (transform.transform, tf_transform);
		for(size_t i=0;i<temp_cloud->size();++i)
		{
			mPointCloudAggregator.addPoints((*temp_cloud)[i], tf_transform);
		}
		publishPointcloud();
	}

	void rotLaserScanCallback( const sensor_msgs::LaserScanPtr& scan)
	{
		ROS_DEBUG("Recieved pointcloud, frame : %s", scan->header.frame_id.c_str());
		geometry_msgs::TransformStamped transform;
		try{
			tfBuffer.canTransform(rootTransform, scan->header.frame_id, scan->header.stamp, ros::Duration(0.2));
			transform = tfBuffer.lookupTransform(rootTransform, scan->header.frame_id, scan->header.stamp);
		}
		catch (tf2::TransformException &ex) {
		  ROS_WARN("%s",ex.what());
		}
		tf::Transform tf_transform;
		tf::transformMsgToTF (transform.transform, tf_transform);
		for (size_t i=0; i < scan->ranges.size(); i++)
		{
			tf::Vector3 pointOut;
			float ang = scan->angle_min+i*scan->angle_increment;
			float dist = scan->ranges[i];
			float intensity = 0;
			if (i <scan->intensities.size())
			{
				intensity = 0.01* scan->intensities[i];
			}

			PointType point;
			point.x = cos(ang)*dist;
			point.y = sin(ang)*dist;
			point.z = 0;

			mPointCloudAggregator.addPoints(point, tf_transform);
		}
		publishPointcloud();
	}


private:
	ros::NodeHandle n;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	std::string rootTransform;
	std::string rotLaserScan;
	std::string rotLaserPointCloud;

	ros::Publisher cloud_pub;
	ros::Publisher progress_pub;
	ros::Publisher done_pub;

	ros::Subscriber rqt_sub;
	ros::Subscriber laserScan_sub;
	ros::Subscriber pointCloud_sub;


	pointCloudAggregator mPointCloudAggregator;
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_aggregator");
	aggregator_node();
}
