#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"


#include "gpu6DSLAM.h"

tf::TransformListener* tf_listener;
ros::Subscriber subscriber_pointcloud2;

std::string frame_global;// = "map";
std::string frame_robot;// = "base_link";

std::string root_folder_name = "/tmp/slam";

gpu6DSLAM slam(root_folder_name);

void callbackPointcloud2(const sensor_msgs::PointCloud2::ConstPtr& msg);
std::string tf_resolve(const std::string& prefix, const std::string& frame_name);

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
	private_node.param<std::string>("topic_pointcloud2", topic_pointcloud2, "/color_pc/output");
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
		//tf_listener->waitForTransform(frame_global, frame_robot, msg->header.stamp,
		//		ros::Duration(1.0));

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


		pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> metascan = slam.getMetascan();

		//publish metascan
		//ToDo
		////////////////////////////



		//for(size_t i = 0 ; i < pc.points.size(); i++)
		//{
		//	std::cout << pc[i].x << " " << pc[i].y << " " << pc[i].z << " " << pc[i].intensity << " " << pc[i].ring << " " << pc[i].normal_x << " " << pc[i].normal_y << " " << pc[i].normal_z << " " << pc[i].label << " " << pc[i].rgb << std::endl;
		//}


	}catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
	}
	//tf_listener->waitForTransform(frame_global, frame_robot, msg->header.stamp,
	//			ros::Duration(1.0));
	//tf_listener->lookupTransform(frame_global, frame_robot, msg->header.stamp,
	//		position_current);



	//std::cout << "jojo" << std::endl;
/*
	if (!mutex.try_lock()) {
		ROS_WARN("mutex.try_lock() failed return");

		//ROS_DEBUG
		//ROS_WARN()
		return;
	}

	tf::StampedTransform position_current;

	tf_listener->waitForTransform(frame_global, frame_laser, msg->header.stamp,
			ros::Duration(1.0));
	tf_listener->lookupTransform(frame_global, frame_laser, msg->header.stamp,
			position_current);

	tfScalar pitch, roll, yaw;
	tf::Matrix3x3(position_current.getRotation()).getRPY(roll, pitch, yaw);
//	////////////we have cloud, x,y,z, yaw, pich, roll
	Eigen::Affine3f mR;
	mR = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
			* Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

	Eigen::Affine3f mT(
			Eigen::Translation3f(position_current.getOrigin().x(),
					position_current.getOrigin().y(),
					position_current.getOrigin().z()));

	Eigen::Affine3f m = mT * mR;

	static double last_odoX = position_current.getOrigin().x();
	static double last_odoY = position_current.getOrigin().y();
	static double last_odoYaw = yaw;

	static Eigen::Affine3f last_mRegistration = m;
	static Eigen::Affine3f last_m = m;

	double tempDist = sqrt(
			(position_current.getOrigin().x() - last_odoX)
					* (position_current.getOrigin().x() - last_odoX)
					+ (position_current.getOrigin().y() - last_odoY)
							* (position_current.getOrigin().y() - last_odoY));

	if (fabs(yaw - last_odoYaw) > angle_threshold
			|| tempDist > distance_threshold) {

		int ni = number_of_iterations_translation;
		if(fabs(yaw - last_odoYaw) > angle_threshold)ni = number_of_iterations_rotation;

		if(!registration(  msg ,		m, last_odoX, last_odoY, last_odoYaw,
										position_current, yaw,last_mRegistration, ni))
		{
			Eigen::Affine3f odometryIncrement = last_m.inverse() * m;
			last_mRegistration = last_mRegistration * odometryIncrement;

			//std::cout << "registration_error" << std::endl;
			//sendDataWithROS("registration_error", last_mRegistration, msg);
			////todo this is error, we are sending odometry increment
			////std::cout << "jojo3" << std::endl;
		}
	}else //if (fabs(yaw - last_odoYaw) > angle_threshold|| tempDist > distance_threshold) {
	{
		Eigen::Affine3f odometryIncrement = last_m.inverse() * m;
		last_mRegistration = last_mRegistration * odometryIncrement;

		//std::cout << "only_odometry" << std::endl;
		sendDataWithROS(last_mRegistration, msg);
	}
//
	last_m = m;
	mutex.unlock();*/
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
