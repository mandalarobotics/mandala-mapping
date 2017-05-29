#include "gpu6DSLAM.h"


void gpu6DSLAM::registerSingleScan(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &pc, Eigen::Affine3f mtf, std::string iso_time_str)
{
	static Eigen::Affine3f last_mtf = mtf;

	Eigen::Affine3f odometryIncrement = last_mtf.inverse() * mtf;
	//last_mRegistration = last_mRegistration * odometryIncrement;


	CCudaWrapper cudaWrapper;
	cudaWrapper.warmUpGPU(this->cudaDevice);

	std::cout << "gpu6DSLAM::registerSingleScan" << std::endl;

	std::string scanName = std::string("scan_") + iso_time_str;
	std::string scanPCDFileName = scanName + std::string(".pcd");

	boost::filesystem::path tfModelFileName = mainPath;
	tfModelFileName/=(std::string("tfModel_" + iso_time_str + ".xml"));
	std::cout << "tfModelFileName: " << tfModelFileName << std::endl;

	boost::filesystem::path processedDataModelFileName = mainPath;
	processedDataModelFileName/=(std::string("tfModelProcessedData_" + iso_time_str + ".xml"));
	std::cout << "processedDataModelFileName: " << processedDataModelFileName << std::endl;

	boost::filesystem::path registeredDataModelFileName = mainPath;
	registeredDataModelFileName/=(std::string("registeredData_" + iso_time_str + ".xml"));
	std::cout << "registeredDataModelFileName: " << registeredDataModelFileName << std::endl;


	boost::filesystem::path rawDataFileName = rawData;
	rawDataFileName/=scanPCDFileName;

	boost::filesystem::path processedDataFileName = processedData;
	processedDataFileName/=scanPCDFileName;

	//save raw data
	std::cout << "raw scan: " << scanName << " will be saved in: " << rawDataFileName << std::endl;
	if(pcl::io::savePCDFileBinary(rawDataFileName.string(), pc) == -1)
	{
		std::cout << "ERROR: problem with saving file: " << rawDataFileName << std::endl;
	}

	//pc processing
	//noise removal
	cudaWrapper.removeNoiseNaive(pc,
			this->noise_removal_resolution,
			this->noise_removal_bounding_box_extension,
			this->noise_removal_number_of_points_in_bucket_threshold);

	//downsampling
	cudaWrapper.downsampling(pc, this->downsampling_resolution, this->downsampling_resolution);

	//semantic classification
	cudaWrapper.classify( pc,
			this->semantic_classification_normal_vectors_search_radius,
			this->semantic_classification_curvature_threshold,
			this->semantic_classification_ground_Z_coordinate_threshold,
			this->semantic_classification_number_of_points_needed_for_plane_threshold,
			this->semantic_classification_bounding_box_extension,
			this->semantic_classification_max_number_considered_in_INNER_bucket,
			this->semantic_classification_max_number_considered_in_OUTER_bucket ) ;

	//save processed data
	std::cout << "processed scan: " << processedDataFileName << " will be saved in: " << processedDataFileName << std::endl;
	if(pcl::io::savePCDFileBinary(processedDataFileName.string(), pc) == -1)
	{
		std::cout << "ERROR: problem with saving file: " << processedDataFileName << std::endl;
	}

	if(vpc.size() == 0)
	{
		this->vpc.push_back(pc);
		this->cloud_ids.push_back(scanName);
		this->vmtf.push_back(mtf);
		this->vmregistered.push_back(mtf);
	}else
	{
		this->vpc.push_back(pc);
		this->cloud_ids.push_back(scanName);
		Eigen::Affine3f m = this->vmtf[this->vmtf.size()-1] * odometryIncrement;
		this->vmtf.push_back(m);


		Eigen::Affine3f mr = this->vmregistered[this->vmregistered.size()-1] * odometryIncrement;
		this->vmregistered.push_back(mr);

		//first step of SLAM - register last arrived scan
		std::cout << "registerLastArrivedScan START" << std::endl;

		//iso_time_str

		pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> pc_before;
		for(size_t i = 0 ;i < vpc.size(); i++)
		{
			pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> pc_before_local = vpc[i];
			transformPointCloud(pc_before_local, this->vmregistered[i]);
			pc_before += pc_before_local;
		}
		pcl::io::savePCDFileBinary(std::string("/tmp/pc_before_")+iso_time_str + std::string(".pcd"), pc_before);


		for(int i = 0 ; i < this->slam_registerLastArrivedScan_number_of_iterations_step1; i++)
		{
			registerLastArrivedScan(cudaWrapper, this->slam_search_radius_step1, this->slam_bucket_size_step1);
		}

		for(int i = 0 ; i < this->slam_registerLastArrivedScan_number_of_iterations_step2; i++)
		{
			registerLastArrivedScan(cudaWrapper, this->slam_search_radius_step2, this->slam_bucket_size_step2);
		}

		for(int i = 0 ; i < this->slam_registerLastArrivedScan_number_of_iterations_step3; i++)
		{
			registerLastArrivedScan(cudaWrapper, this->slam_search_radius_step3, this->slam_bucket_size_step3);
		}

		pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> pc_after;
		for(size_t i = 0 ;i < vpc.size(); i++)
		{
			pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> pc_before_local = vpc[i];
			transformPointCloud(pc_before_local, this->vmregistered[i]);
			pc_after += pc_before_local;
		}
		pcl::io::savePCDFileBinary(std::string("/tmp/pc_after_")+iso_time_str + std::string(".pcd"), pc_after);



		std::cout << "registerLastArrivedScan FINISHED" << std::endl;
	}

	this->tfModel.setAffine(cloud_ids[cloud_ids.size()-1], mtf.matrix());
	this->tfModel.setPointcloudName(cloud_ids[cloud_ids.size()-1], scanPCDFileName);
	this->tfModel.saveFile(tfModelFileName.string());

	this->processedDataModel.setAffine(cloud_ids[cloud_ids.size()-1], mtf.matrix());
	this->processedDataModel.setPointcloudName(cloud_ids[cloud_ids.size()-1], scanPCDFileName);
	this->processedDataModel.saveFile(processedDataModelFileName.string());

	this->registeredModel.setPointcloudName(cloud_ids[cloud_ids.size()-1], scanPCDFileName);
	for(size_t i = 0 ; i < this->vmregistered.size(); i++)
	{
		this->registeredModel.setAffine(cloud_ids[i], vmregistered[i].matrix());
	}
	this->registeredModel.saveFile(registeredDataModelFileName.string());


	last_mtf = mtf;

	return;
}


pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> gpu6DSLAM::getMetascan()
{
	std::cout << "gpu6DSLAM::getMetascan" << std::endl;

	pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> metascan;

	return metascan;
}

void gpu6DSLAM::registerLastArrivedScan(CCudaWrapper &cudaWrapper, float slam_search_radius, float slam_bucket_size)
{
	//this->slam_registerLastArrivedScan_distance_threshold = 10.0f;

	std::vector<Eigen::Affine3f> v_poses;

	size_t i = vmregistered.size() - 1;

	Eigen::Vector3f omfika1;
	Eigen::Vector3f xyz1;
	Eigen::Affine3f pose1;

	cudaWrapper.Matrix4ToEuler(vmregistered[i], omfika1, xyz1);
	cudaWrapper.EulerToMatrix(omfika1, xyz1, pose1);

	pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> _point_cloud_1 = vpc[i];
	transformPointCloud(_point_cloud_1, pose1);

	observations_t obs;
	obs.om = omfika1.x();
	obs.fi = omfika1.y();
	obs.ka = omfika1.z();
	obs.tx = xyz1.x();
	obs.ty = xyz1.y();
	obs.tz = xyz1.z();

	for(size_t j = 0; j < i; j++)
	{
		Eigen::Vector3f omfika2;
		Eigen::Vector3f xyz2;
		Eigen::Affine3f pose2;
		cudaWrapper.Matrix4ToEuler(vmregistered[j], omfika2, xyz2);
		cudaWrapper.EulerToMatrix(omfika2, xyz2, pose2);

		float dist = sqrt(  (xyz1.x() - xyz2.x()) * (xyz1.x() - xyz2.x()) +
							(xyz1.y() - xyz2.y()) * (xyz1.y() - xyz2.y()) +
							(xyz1.z() - xyz2.z()) * (xyz1.z() - xyz2.z()) );

		std::cout << "dist: " << dist << std::endl;

		if(dist < this->slam_registerLastArrivedScan_distance_threshold)
		{
			pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> _point_cloud_2 = vpc[j];
			transformPointCloud(_point_cloud_2, pose2);

			std::vector<int> nearest_neighbour_indexes;
			nearest_neighbour_indexes.resize(_point_cloud_2.size());
			std::fill(nearest_neighbour_indexes.begin(), nearest_neighbour_indexes.end(), -1);

			cudaWrapper.semanticNearestNeighbourhoodSearch(
									_point_cloud_1,
									_point_cloud_2,
									slam_search_radius,
									slam_bucket_size,
									this->slam_bounding_box_extension,
									this->slam_max_number_considered_in_INNER_bucket,
									this->slam_max_number_considered_in_OUTER_bucket,
									nearest_neighbour_indexes);

			int number_of_observations_plane = 0;
			int number_of_observations_edge = 0;
			int number_of_observations_ceiling = 0;
			int number_of_observations_ground = 0;

			for(size_t ii = 0 ; ii < nearest_neighbour_indexes.size(); ii++)
			{
				if(nearest_neighbour_indexes[ii] != -1)
				{
					switch (_point_cloud_2[ii].label)
					{
						case LABEL_PLANE:
						{
							number_of_observations_plane ++;
							break;
						}
						case LABEL_EDGE:
						{
							number_of_observations_edge ++;
							break;
						}
						case LABEL_CEILING:
						{
							number_of_observations_ceiling ++;
							break;
						}
						case LABEL_GROUND:
						{
							number_of_observations_ground ++;
							break;
						}
					}
				}
			}


			for(size_t ii = 0 ; ii < nearest_neighbour_indexes.size(); ii++)
			{
				if(nearest_neighbour_indexes[ii] != -1)
				{
					pcl::PointXYZ p1(_point_cloud_1[nearest_neighbour_indexes[ii]].x, _point_cloud_1[nearest_neighbour_indexes[ii]].y, _point_cloud_1[nearest_neighbour_indexes[ii]].z);
					pcl::PointXYZ p2(_point_cloud_2[ii].x, _point_cloud_2[ii].y, _point_cloud_2[ii].z);

					obs_nn_t obs_nn;
					obs_nn.x0 = vpc[i].points[nearest_neighbour_indexes[ii]].x;
					obs_nn.y0 = vpc[i].points[nearest_neighbour_indexes[ii]].y;
					obs_nn.z0 = vpc[i].points[nearest_neighbour_indexes[ii]].z;
					obs_nn.x_diff = p1.x - p2.x;
					obs_nn.y_diff = p1.y - p2.y;
					obs_nn.z_diff = p1.z - p2.z;
					switch(_point_cloud_2[ii].label)
					{
						case 0://plane
						{
							obs_nn.P = slam_observation_weight_plane/number_of_observations_plane;
							break;
						}
						case 1: //edge
						{
							obs_nn.P = slam_observation_weight_edge/number_of_observations_edge;
							break;
						}
						case 2: //ceiling
						{
							obs_nn.P = slam_observation_weight_ceiling/number_of_observations_ceiling;
							break;
						}
						case 3: //floor/ground
						{
							obs_nn.P = slam_observation_weight_ground/number_of_observations_ground;
							break;
						}
					}
					obs.vobs_nn.push_back(obs_nn);
				}
			}

			std::cout << "obs.vobs_nn.size(): " << obs.vobs_nn.size() << std::endl;

			if(obs.vobs_nn.size() > this->slam_number_of_observations_threshold )
			{
			//std::cout << "obs.vobs_nn.size(): " << obs.vobs_nn.size() << std::endl;
				if(cudaWrapper.registerLS(obs))
				{
					Eigen::Vector3f omfika1_res(obs.om, obs.fi, obs.ka);
					Eigen::Vector3f xyz1_res(obs.tx, obs.ty, obs.tz);
					Eigen::Affine3f pose1_res;
					cudaWrapper.EulerToMatrix(omfika1_res, xyz1_res, pose1_res);

					vmregistered[i] = pose1_res;
				}
			}
		}
	}
}

void gpu6DSLAM::transformPointCloud(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &pointcloud, Eigen::Affine3f transform)
{
	for(size_t i = 0; i < pointcloud.size(); i++)
	{
		Eigen::Vector3f p(pointcloud[i].x, pointcloud[i].y, pointcloud[i].z);
		Eigen::Vector3f pt;

		pt = transform * p;

		pointcloud[i].x = pt.x();
		pointcloud[i].y = pt.y();
		pointcloud[i].z = pt.z();

		Eigen::Affine3f tr = transform;

		tr(0,3) = 0.0f;
		tr(1,3) = 0.0f;
		tr(2,3) = 0.0f;

		Eigen::Vector3f n(pointcloud[i].normal_x, pointcloud[i].normal_y, pointcloud[i].normal_z);
		Eigen::Vector3f nt;

		nt = tr * n;
		pointcloud[i].normal_x = nt.x();
		pointcloud[i].normal_y = nt.y();
		pointcloud[i].normal_z = nt.z();
	}
	return;
}
