#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"


#include "gpu6DSLAM.h"

tf::TransformListener* tf_listener;
ros::Subscriber subscriber_pointcloud2;
ros::Publisher publisher_metascan;

std::string frame_global;// = "map";
std::string frame_robot;// = "base_link";
std::string frame_map = "/map";
std::string root_folder_name = "/tmp/slam";

gpu6DSLAM slam(root_folder_name);

void callbackPointcloud2(const sensor_msgs::PointCloud2::ConstPtr& msg);
std::string tf_resolve(const std::string& prefix, const std::string& frame_name);

//

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gpu6dslam_node");
	ros::NodeHandle private_node("~");
	ros::NodeHandle public_node("");

	tf_listener = new tf::TransformListener();

	////////////////////////////////////////////////////////////////////////
	ROS_INFO("reading parameters");
	std::string topic_pointcloud2;
	//private_node.param<std::string>("topic_pointcloud2", topic_pointcloud2, "/unit_sync/stopScanOutput");
	//private_node.param<std::string>("topic_pointcloud2", topic_pointcloud2, "/color_pc/output");
	private_node.param<std::string>("topic_pointcloud2", topic_pointcloud2, "/m3d_test/aggregator/cloud");

	ROS_INFO("param topic_pointcloud2: '%s'", topic_pointcloud2.c_str());

	std::string tf_prefix = tf::getPrefixParam(public_node);
	ROS_INFO("param tf_prefix: '%s'", tf_prefix.c_str());

	private_node.param<std::string>("frame_global", frame_global, "odom");
	frame_global = tf_resolve(tf_prefix, frame_global);
	ROS_INFO("param frame_global: '%s'", frame_global.c_str());

	private_node.param<std::string>("frame_robot", frame_robot, "base_link");
	frame_robot = tf_resolve(tf_prefix, frame_robot);
	ROS_INFO("param frame_robot: '%s'", frame_robot.c_str());

	///////////////////////////////////////////////////////////////////////////

	ROS_INFO("setting up topic communication");
	subscriber_pointcloud2 = public_node.subscribe(topic_pointcloud2, 1,
			callbackPointcloud2);

	publisher_metascan = public_node.advertise<sensor_msgs::PointCloud2>("/metascan", 1);

	while (ros::ok())
	//&& (!tf_listener->waitForTransform(frame_global, frame_robot,
	//					ros::Time(), ros::Duration(1.0))))
	{
		ros::spinOnce ();
	}

	std::cout << "!ros::ok() return 0" << std::endl;
	return 0;
}

void callbackPointcloud2(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	std::cout << "callbackPointcloud2" << std::endl;

	tf::StampedTransform position_current;
	try
	{
		tf_listener->waitForTransform(frame_global, msg->header.frame_id, msg->header.stamp,
				ros::Duration(1.0));

		//tf_listener->lookupTransform(frame_global, frame_robot, msg->header.stamp,
		//		position_current);

		tf_listener->lookupTransform(frame_global, /*frame_robot*/ msg->header.frame_id, msg->header.stamp, position_current);


        Eigen::Affine3d dm = Eigen::Affine3d::Identity();

        tf::transformTFToEigen (position_current, dm);

        Eigen::Affine3f m = dm.cast<float>();

		pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> pc;
		pcl::fromROSMsg(*msg, pc);

		try
		{
			std::stringstream ss;
		    ss << msg->header.stamp.sec;

			slam.registerSingleScan(pc, m, ss.str());
		}catch (thrust::system_error e)
		{
			 std::cerr << "Error: " << e.what() << std::endl;
		}




		if(publisher_metascan.getNumSubscribers() > 0)
		{
			pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> metascan = slam.getMetascan(m);
			std::cout << "Size metascan: " << metascan.size() << std::endl;

			pcl::PCLPointCloud2 pcl_pc2;
			pcl::toPCLPointCloud2(metascan,pcl_pc2);
			sensor_msgs::PointCloud2 cloud;
			pcl_conversions::fromPCL(pcl_pc2,cloud);

			cloud.header.frame_id = frame_global;//frame_map;// TODO
			cloud.header.stamp = ros::Time::now();

			publisher_metascan.publish(cloud);
			ROS_INFO("Publish metascan done");
		}

		////////////////////////////



		//for(size_t i = 0 ; i < pc.points.size(); i++)
		//{
		//	std::cout << pc[i].x << " " << pc[i].y << " " << pc[i].z << " " << pc[i].intensity << " " << pc[i].ring << " " << pc[i].normal_x << " " << pc[i].normal_y << " " << pc[i].normal_z << " " << pc[i].label << " " << pc[i].rgb << std::endl;
		//}


	}catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
	}

}

std::string tf_resolve(const std::string& prefix,
		const std::string& frame_name) {
	if (frame_name.size() > 0)
		if (frame_name[0] == '/') {
			return frame_name;
		}
	if (prefix.size() > 0) {
		if (prefix[0] == '/') {
			std::string composite = prefix;
			composite.append("/");
			composite.append(frame_name);
			return composite;

		} else {
			std::string composite;
			composite = "/";
			composite.append(prefix);
			composite.append("/");
			composite.append(frame_name);
			return composite;
		}
	} else {
		std::string composite;
		composite = "/";
		composite.append(frame_name);
		return composite;
	}
}
