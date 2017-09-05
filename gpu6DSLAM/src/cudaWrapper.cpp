#include "cudaWrapper.h"


CCudaWrapper::CCudaWrapper()
{
	this->d_point_cloud = 0;
	this->d_hashTable = 0;
	this->d_buckets = 0;
	this->d_markers = 0;
	this->d_mean = 0;
	this->d_first_point_cloud = 0;
	this->d_second_point_cloud = 0;
	this->d_nearest_neighbour_indexes = 0;
	this->d_A = 0;
	this->d_P = 0;
	this->d_l = 0;
	this->d_obs_nn = 0;
}

CCudaWrapper::~CCudaWrapper()
{
	cudaFree(this->d_point_cloud); this->d_point_cloud = 0;
	cudaFree(this->d_hashTable); this->d_hashTable = 0;
	cudaFree(this->d_buckets); this->d_buckets = 0;
	cudaFree(this->d_markers); this->d_markers = 0;
	cudaFree(this->d_mean); this->d_mean = 0;
	cudaFree(this->d_first_point_cloud); this->d_first_point_cloud = 0;
	cudaFree(this->d_second_point_cloud); this->d_second_point_cloud = 0;
	cudaFree(this->d_nearest_neighbour_indexes); this->d_nearest_neighbour_indexes = 0;
	cudaFree(this->d_A); this->d_A = 0;
	cudaFree(this->d_P); this->d_P = 0;
	cudaFree(this->d_l); this->d_l = 0;
	cudaFree(this->d_obs_nn); this->d_obs_nn = 0;
}

void CCudaWrapper::warmUpGPU(int cudaDevice)
{
	cudaError_t errCUDA = ::cudaSuccess;
	errCUDA = cudaSetDevice(cudaDevice);
	throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaWarmUpGPU();
	throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	getNumberOfAvailableThreads(cudaDevice, this->threads, this->threadsNV);//ToDo what if false

return;
}

int CCudaWrapper::getNumberOfAvailableThreads(int cudaDevice)
{
	cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, cudaDevice);

	int threads = 0;
	if(prop.major == 2)
	{
		threads=prop.maxThreadsPerBlock/2;
	}else if(prop.major > 2)
	{
		threads=prop.maxThreadsPerBlock;
	}else
	{
		return 0;
	}

	return threads;
}

bool CCudaWrapper::getNumberOfAvailableThreads(int cudaDevice, int &threads, int &threadsNV)
{
	cudaDeviceProp prop;
	cudaGetDeviceProperties(&prop, cudaDevice);

	threads = 0;
	threadsNV = 0;
	if(prop.major == 2)
	{
		threads=prop.maxThreadsPerBlock/2;
		threadsNV=prop.maxThreadsPerBlock/8;
	}else if(prop.major > 2)
	{
		threads=prop.maxThreadsPerBlock;
		threadsNV=prop.maxThreadsPerBlock/4;
	}else
	{
		return false;
	}
	return true;
}

void CCudaWrapper::coutMemoryStatus()
{
	size_t free_byte ;
    size_t total_byte ;

    cudaError_t err = cudaMemGetInfo( &free_byte, &total_byte ) ;

    if(err != ::cudaSuccess)
	{
		std::cout << "Error: cudaMemGetInfo fails: " << cudaGetErrorString(err) << std::endl;
		return;
	}
    double free_db = (double)free_byte ;
    double total_db = (double)total_byte ;
    double used_db = total_db - free_db ;

    std::cout << "GPU memory usage: used = " <<
    		used_db/1024.0/1024.0 <<
			"(MB), free = " <<
			free_db/1024.0/1024.0 <<
			"(MB), total = " <<
			total_db/1024.0/1024.0 <<
			"(MB)" << std::endl;
}

void CCudaWrapper::removeNoiseNaive(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &point_cloud, float resolution, float bounding_box_extension,  int number_of_points_in_bucket_threshold)
{
	cudaError_t errCUDA = ::cudaSuccess;
	gridParameters rgd_params;

	bool* h_markers = 0;

	errCUDA = cudaMalloc((void**)&this->d_point_cloud, point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_point_cloud, point_cloud.points.data(), point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGridParams(this->d_point_cloud, point_cloud.points.size(),
			resolution, resolution, resolution, bounding_box_extension, rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_hashTable, point_cloud.points.size()*sizeof(hashElement));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_buckets, rgd_params.number_of_buckets*sizeof(bucket));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGrid(this->threads, this->d_point_cloud, this->d_buckets, this->d_hashTable, point_cloud.points.size(), rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_markers, point_cloud.points.size()*sizeof(bool) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaRemoveNoiseNaive(this->threads, this->d_markers, this->d_point_cloud, this->d_hashTable, this->d_buckets, rgd_params, point_cloud.points.size(), number_of_points_in_bucket_threshold);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	h_markers = (bool *)malloc(point_cloud.points.size()*sizeof(bool));

	errCUDA = cudaMemcpy(h_markers, this->d_markers, point_cloud.points.size()*sizeof(bool),cudaMemcpyDeviceToHost);
		if(errCUDA != ::cudaSuccess)
		{
			free(h_markers);
		}
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> filtered_point_cloud;
	for(size_t i = 0; i < point_cloud.points.size(); i++)
	{
		if(h_markers[i])filtered_point_cloud.push_back(point_cloud[i]);
	}

	point_cloud = filtered_point_cloud;

	free(h_markers);

	errCUDA = cudaFree(d_point_cloud); d_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_hashTable); d_hashTable = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_buckets); d_buckets = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_markers); d_markers = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	return;
}

void CCudaWrapper::downsampling(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &point_cloud, float resolution, float bounding_box_extension)
{
	cudaError_t errCUDA = ::cudaSuccess;

	gridParameters rgd_params;
	bool* h_markers = 0;

	errCUDA = cudaMalloc((void**)&this->d_point_cloud, point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_point_cloud, point_cloud.points.data(), point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGridParams(this->d_point_cloud, point_cloud.points.size(),
			resolution, resolution, resolution, bounding_box_extension, rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

		std::cout << "regular grid parameters for normal vectors:" << std::endl;
		std::cout << "bounding_box_min_X: " << rgd_params.bounding_box_min_X << std::endl;
		std::cout << "bounding_box_min_Y: " << rgd_params.bounding_box_min_Y << std::endl;
		std::cout << "bounding_box_min_Z: " << rgd_params.bounding_box_min_Z << std::endl;
		std::cout << "bounding_box_max_X: " << rgd_params.bounding_box_max_X << std::endl;
		std::cout << "bounding_box_max_Y: " << rgd_params.bounding_box_max_Y << std::endl;
		std::cout << "bounding_box_max_Z: " << rgd_params.bounding_box_max_Z << std::endl;
		std::cout << "number_of_buckets_X: " << rgd_params.number_of_buckets_X << std::endl;
		std::cout << "number_of_buckets_Y: " << rgd_params.number_of_buckets_Y << std::endl;
		std::cout << "number_of_buckets_Z: " << rgd_params.number_of_buckets_Z << std::endl;
		std::cout << "resolution_X: " << rgd_params.resolution_X << std::endl;
		std::cout << "resolution_Y: " << rgd_params.resolution_Y << std::endl;
		std::cout << "resolution_Z: " << rgd_params.resolution_Z << std::endl;

	errCUDA = cudaMalloc((void**)&this->d_hashTable,point_cloud.points.size()*sizeof(hashElement));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_buckets, rgd_params.number_of_buckets*sizeof(bucket));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGrid(this->threads, this->d_point_cloud, this->d_buckets, this->d_hashTable, point_cloud.points.size(), rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_markers, point_cloud.points.size()*sizeof(bool) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaDownSample(this->threads, this->d_markers, this->d_hashTable, this->d_buckets, rgd_params, point_cloud.points.size());
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	h_markers = (bool *)malloc(point_cloud.points.size()*sizeof(bool));

	errCUDA = cudaMemcpy(h_markers, this->d_markers, point_cloud.points.size()*sizeof(bool),cudaMemcpyDeviceToHost);
		if(errCUDA != ::cudaSuccess)
		{
			free(h_markers);
		}
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> downsampled_point_cloud;
	for(size_t i = 0; i < point_cloud.points.size(); i++)
	{
		if(h_markers[i])downsampled_point_cloud.push_back(point_cloud[i]);
	}

	point_cloud = downsampled_point_cloud;

	free(h_markers);

	errCUDA = cudaFree(d_point_cloud); d_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_hashTable); d_hashTable = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_buckets); d_buckets = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_markers); d_markers = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	return;
}

void CCudaWrapper::classify(  pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB>	&point_cloud,
					float normal_vectors_search_radius,
					float curvature_threshold,
					float ground_Z_coordinate_threshold,
					int number_of_points_needed_for_plane_threshold,
					float bounding_box_extension,
					int max_number_considered_in_INNER_bucket,
					int max_number_considered_in_OUTER_bucket  )
{
	gridParameters rgd_params;
	cudaError_t errCUDA = ::cudaSuccess;

	errCUDA = cudaMalloc((void**)&this->d_point_cloud, point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_point_cloud, point_cloud.points.data(), point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGridParams(this->d_point_cloud, point_cloud.points.size(),
			normal_vectors_search_radius, normal_vectors_search_radius, normal_vectors_search_radius, bounding_box_extension, rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_hashTable,point_cloud.points.size()*sizeof(hashElement));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_buckets, rgd_params.number_of_buckets*sizeof(bucket));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGrid(this->threadsNV, this->d_point_cloud, this->d_buckets, this->d_hashTable, point_cloud.points.size(), rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_mean, point_cloud.points.size()*sizeof(simple_point3D) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaSemanticLabelingPlaneEdges(
			this->threadsNV,
			this->d_point_cloud,
			point_cloud.size(),
			this->d_hashTable,
			this->d_buckets,
			this->d_mean,
			rgd_params,
			normal_vectors_search_radius,
			max_number_considered_in_INNER_bucket,
			max_number_considered_in_OUTER_bucket,
			curvature_threshold,
			number_of_points_needed_for_plane_threshold);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaSemanticLabelingFloorCeiling(
			this->threads,
			this->d_point_cloud,
			point_cloud.size(),
			ground_Z_coordinate_threshold);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(point_cloud.points.data(), this->d_point_cloud, point_cloud.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyDeviceToHost);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_point_cloud); d_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_hashTable); d_hashTable = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_buckets); d_buckets = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_mean); d_mean = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	return ;
}

void CCudaWrapper::semanticNearestNeighbourhoodSearch(
						pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &first_point_cloud,
						pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &second_point_cloud,
						float search_radius,
						float bucket_size,
						float bounding_box_extension,
						int max_number_considered_in_INNER_bucket,
						int max_number_considered_in_OUTER_bucket,
						std::vector<int> &nearest_neighbour_indexes)
{
	if(nearest_neighbour_indexes.size() != second_point_cloud.size())return;

	gridParameters rgd_params;
	cudaError_t errCUDA = ::cudaSuccess;


	errCUDA = cudaMalloc((void**)&this->d_first_point_cloud, first_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_first_point_cloud, first_point_cloud.points.data(), first_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_second_point_cloud, second_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_second_point_cloud, second_point_cloud.points.data(), second_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGridParams(this->d_first_point_cloud, first_point_cloud.points.size(),
			bucket_size, bucket_size, bucket_size, bounding_box_extension, rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_hashTable, first_point_cloud.points.size()*sizeof(hashElement));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_buckets, rgd_params.number_of_buckets*sizeof(bucket));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_nearest_neighbour_indexes, second_point_cloud.points.size()*sizeof(int));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);


	errCUDA = cudaCalculateGrid(this->threads, this->d_first_point_cloud, this->d_buckets, this->d_hashTable, first_point_cloud.points.size(), rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaSemanticNearestNeighborSearch(
			this->threads,
			this->d_first_point_cloud,
			first_point_cloud.points.size(),
			this->d_second_point_cloud,
			second_point_cloud.points.size(),
			this->d_hashTable,
			this->d_buckets,
			rgd_params,
			search_radius,
			max_number_considered_in_INNER_bucket,
			max_number_considered_in_OUTER_bucket,
			this->d_nearest_neighbour_indexes);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(nearest_neighbour_indexes.data(), this->d_nearest_neighbour_indexes, second_point_cloud.points.size()*sizeof(int),cudaMemcpyDeviceToHost);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_first_point_cloud); d_first_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_second_point_cloud); d_second_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_hashTable); d_hashTable = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_buckets); d_buckets = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_nearest_neighbour_indexes); d_nearest_neighbour_indexes = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);


	return;
}


void CCudaWrapper::Matrix4ToEuler(const double *alignxf, double *rPosTheta, double *rPos)
{
	double _trX, _trY;

	if(alignxf[0] > 0.0)
	{
		rPosTheta[1] = asin(alignxf[8]);
	}
	else
	{
		rPosTheta[1] = M_PI - asin(alignxf[8]);
	}

	double  C    =  cos( rPosTheta[1] );
	if ( fabs( C ) > 0.005 )
	{                 // Gimball lock?
		_trX      =  alignxf[10] / C;             // No, so get X-axis angle
		_trY      =  -alignxf[9] / C;
		rPosTheta[0]  = atan2( _trY, _trX );
		_trX      =  alignxf[0] / C;              // Get Z-axis angle
		_trY      = -alignxf[4] / C;
		rPosTheta[2]  = atan2( _trY, _trX );
	}
	else
	{                                    // Gimball lock has occurred
		rPosTheta[0] = 0.0;                       // Set X-axis angle to zero
		_trX      =  alignxf[5];  //1                // And calculate Z-axis angle
		_trY      =  alignxf[1];  //2
		rPosTheta[2]  = atan2( _trY, _trX );
	}

	rPosTheta[0] = rPosTheta[0];
	rPosTheta[1] = rPosTheta[1];
	rPosTheta[2] = rPosTheta[2];

	if (rPos != 0)
	{
		rPos[0] = alignxf[12];
		rPos[1] = alignxf[13];
		rPos[2] = alignxf[14];
	}
}

void CCudaWrapper::Matrix4ToEuler(Eigen::Affine3f m, Eigen::Vector3f &omfika, Eigen::Vector3f &xyz)
{
	double _trX, _trY;

	if(m(0,0) > 0.0)
	{
		omfika.y() = asin(m(0,2));
	}
	else
	{
		omfika.y() = M_PI - asin(m(0,2));
	}

	double  C    =  cos( omfika.y() );
	if ( fabs( C ) > 0.005 )
	{                 // Gimball lock?
		_trX      =  m(2,2) / C;             // No, so get X-axis angle
		_trY      =  -m(1,2) / C;
		omfika.x()  = atan2( _trY, _trX );
		_trX      =  m(0,0) / C;              // Get Z-axis angle
		_trY      = -m(0,1) / C;
		omfika.z()= atan2( _trY, _trX );
	}
	else
	{                                    // Gimball lock has occurred
		omfika.x() = 0.0;                       // Set X-axis angle to zero
		_trX      =  m(1,1);  //1                // And calculate Z-axis angle
		_trY      =  m(1,0);  //2
		omfika.z() = atan2( _trY, _trX );
	}

	xyz.x() = m(0,3);
	xyz.y() = m(1,3);
	xyz.z() = m(2,3);
}

void CCudaWrapper::EulerToMatrix(Eigen::Vector3f omfika, Eigen::Vector3f xyz, Eigen::Affine3f &m)
{
	Eigen::Affine3f mR;
	mR = Eigen::AngleAxisf(omfika.x(), Eigen::Vector3f::UnitX())
		  * Eigen::AngleAxisf(omfika.y(), Eigen::Vector3f::UnitY())
		  * Eigen::AngleAxisf(omfika.z(), Eigen::Vector3f::UnitZ());
	Eigen::Affine3f mT(Eigen::Translation3f(xyz.x(), xyz.y(), xyz.z()));
	m = mT * mR;
}

bool CCudaWrapper::registerLS(observations_t &obs)
{
	cudaError_t errCUDA = ::cudaSuccess;
	CCUDA_AX_B_SolverWrapper::Solver_Method solver_method = CCUDA_AX_B_SolverWrapper::chol;

	double x[6];

	errCUDA  = cudaMalloc((void**)&d_A,  obs.vobs_nn.size() * 3 * 6 *sizeof(double));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA  = cudaMalloc((void**)&d_P,  obs.vobs_nn.size() * 3 *sizeof(double));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA  = cudaMalloc((void**)&d_l,  obs.vobs_nn.size() * 3 *sizeof(double));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&d_obs_nn, obs.vobs_nn.size()*sizeof(obs_nn_t) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(d_obs_nn, obs.vobs_nn.data(), obs.vobs_nn.size()*sizeof(obs_nn_t), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA =  fill_A_l_cuda(threads, d_A, obs.tx, obs.ty, obs.tz, obs.om, obs.fi, obs.ka, d_obs_nn, obs.vobs_nn.size(),
							d_P, d_l);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	CCUDA_AX_B_SolverWrapper * wr = new CCUDA_AX_B_SolverWrapper(false, 0);

	CCUDA_AX_B_SolverWrapper::CCUDA_AX_B_SolverWrapper_error errAXB =
					wr->Solve_ATPA_ATPl_x_data_on_GPU(this->threads, d_A, d_P, d_l, x, 6, obs.vobs_nn.size() * 3, solver_method);
	if(errAXB!=CCUDA_AX_B_SolverWrapper::success)
	{
		std::cout << "problem with solving Ax=B" << std::endl;
		delete wr;

		return false;
	}

	delete wr;

	errCUDA = cudaFree(d_obs_nn); d_obs_nn = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	std::cout << "Ax=B solution:" << std::endl;
	for(int i = 0 ; i < 6; i++)
	{
		std::cout << "x[" << i <<"]: " << x[i] << std::endl;
	}

	errCUDA = cudaFree(d_A); d_A = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_P); d_P = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_l); d_l = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	obs.tx += x[0];
	obs.ty += x[1];
	obs.tz += x[2];
	obs.om += x[3];
	obs.fi += x[4];
	obs.ka += x[5];
	return true;
}

bool CCudaWrapper::registerLS_4DOF(observations_t &obs)
{
	cudaError_t errCUDA = ::cudaSuccess;
	CCUDA_AX_B_SolverWrapper::Solver_Method solver_method = CCUDA_AX_B_SolverWrapper::chol;

	double x[4];

	errCUDA  = cudaMalloc((void**)&d_A,  obs.vobs_nn.size() * 3 * 4 *sizeof(double));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA  = cudaMalloc((void**)&d_P,  obs.vobs_nn.size() * 3 *sizeof(double));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA  = cudaMalloc((void**)&d_l,  obs.vobs_nn.size() * 3 *sizeof(double));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&d_obs_nn, obs.vobs_nn.size()*sizeof(obs_nn_t) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(d_obs_nn, obs.vobs_nn.data(), obs.vobs_nn.size()*sizeof(obs_nn_t), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA =  fill_A_l_4DOFcuda(threads, d_A, obs.tx, obs.ty, obs.tz, obs.om, obs.fi, obs.ka, d_obs_nn, obs.vobs_nn.size(),
							d_P, d_l);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	CCUDA_AX_B_SolverWrapper * wr = new CCUDA_AX_B_SolverWrapper(false, 0);

	CCUDA_AX_B_SolverWrapper::CCUDA_AX_B_SolverWrapper_error errAXB =
					wr->Solve_ATPA_ATPl_x_data_on_GPU(this->threads, d_A, d_P, d_l, x, 4, obs.vobs_nn.size() * 3, solver_method);
	if(errAXB!=CCUDA_AX_B_SolverWrapper::success)
	{
		std::cout << "problem with solving Ax=B" << std::endl;
		delete wr;

		return false;
	}

	delete wr;

	errCUDA = cudaFree(d_obs_nn); d_obs_nn = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	std::cout << "Ax=B solution:" << std::endl;
	for(int i = 0 ; i < 4; i++)
	{
		std::cout << "x[" << i <<"]: " << x[i] << std::endl;
	}

	errCUDA = cudaFree(d_A); d_A = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_P); d_P = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(d_l); d_l = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	obs.tx += x[0];
	obs.ty += x[1];
	obs.tz += x[2];
	//obs.om += x[3];
	//obs.fi += x[4];
	obs.ka += x[3];
	return true;
}

void CCudaWrapper::throw_on_cuda_error(cudaError_t code, const char *file, int line)
{
  if(code != cudaSuccess)
  {
    std::stringstream ss;
    ss << file << "(" << line << ")";
    std::string file_and_line;
    ss >> file_and_line;
    throw thrust::system_error(code, thrust::cuda_category(), file_and_line);
  }
}

void CCudaWrapper::findBestYaw(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &first_point_cloud,
				Eigen::Affine3f first_transform,
				pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &second_point_cloud,
				Eigen::Affine3f second_transform,
				float bucket_size,
				float bounding_box_extension,
				float search_radius,
				int max_number_considered_in_INNER_bucket,
				int max_number_considered_in_OUTER_bucket,
				float angle_start,
				float angle_finish,
				float angle_step,
				Eigen::Affine3f &myaw)
{
	gridParameters rgd_params;
	cudaError_t errCUDA = ::cudaSuccess;


	errCUDA = cudaMalloc((void**)&this->d_first_point_cloud, first_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_first_point_cloud, first_point_cloud.points.data(), first_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_second_point_cloud, second_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMemcpy(this->d_second_point_cloud, second_point_cloud.points.data(), second_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB), cudaMemcpyHostToDevice);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	Eigen::Affine3f matrix  = second_transform;
	float r00 = matrix.matrix()(0, 0);
	float r10 = matrix.matrix()(1, 0);
	float r20 = matrix.matrix()(2, 0);

	float r01 = matrix.matrix()(0, 1);
	float r11 = matrix.matrix()(1, 1);
	float r21 = matrix.matrix()(2, 1);

	float r02 = matrix.matrix()(0, 2);
	float r12 = matrix.matrix()(1, 2);
	float r22 = matrix.matrix()(2, 2);

	float t0 = matrix.matrix()(0, 3);
	float t1 = matrix.matrix()(1, 3);
	float t2 = matrix.matrix()(2, 3);

	errCUDA = cudaTransformPointCloud(threads, this->d_second_point_cloud, second_point_cloud.points.size(), r00, r10, r20, r01, r11, r21, r02, r12, r22, t0, t1, t2);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	matrix = first_transform.inverse();
	r00 = matrix.matrix()(0, 0);
	r10 = matrix.matrix()(1, 0);
	r20 = matrix.matrix()(2, 0);

	r01 = matrix.matrix()(0, 1);
	r11 = matrix.matrix()(1, 1);
	r21 = matrix.matrix()(2, 1);

	r02 = matrix.matrix()(0, 2);
	r12 = matrix.matrix()(1, 2);
	r22 = matrix.matrix()(2, 2);

	t0 = matrix.matrix()(0, 3);
	t1 = matrix.matrix()(1, 3);
	t2 = matrix.matrix()(2, 3);

	errCUDA = cudaTransformPointCloud(threads, this->d_second_point_cloud, second_point_cloud.points.size(), r00, r10, r20, r01, r11, r21, r02, r12, r22, t0, t1, t2);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);


	errCUDA = cudaMalloc((void**)&this->d_point_cloud, second_point_cloud.points.size()*sizeof(lidar_pointcloud::PointXYZIRNLRGB) );
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaCalculateGridParams(this->d_first_point_cloud, first_point_cloud.points.size(),
			bucket_size, bucket_size, bucket_size, bounding_box_extension, rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_hashTable, first_point_cloud.points.size()*sizeof(hashElement));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_buckets, rgd_params.number_of_buckets*sizeof(bucket));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaMalloc((void**)&this->d_nearest_neighbour_indexes, second_point_cloud.points.size()*sizeof(int));
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);


	errCUDA = cudaCalculateGrid(this->threads, this->d_first_point_cloud, this->d_buckets, this->d_hashTable, first_point_cloud.points.size(), rgd_params);
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	int max_number_of_nn = 0;
	int angle_res = 0;

	for(float i = angle_start; i <= angle_finish; i+=angle_step)
	{
		float anglaRad = i*M_PI/180.0;

		matrix = Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitX())
								  * Eigen::AngleAxisf(0.0f, Eigen::Vector3f::UnitY())
								  * Eigen::AngleAxisf(anglaRad, Eigen::Vector3f::UnitZ());

		r00 = matrix.matrix()(0, 0);
		r10 = matrix.matrix()(1, 0);
		r20 = matrix.matrix()(2, 0);

		r01 = matrix.matrix()(0, 1);
		r11 = matrix.matrix()(1, 1);
		r21 = matrix.matrix()(2, 1);

		r02 = matrix.matrix()(0, 2);
		r12 = matrix.matrix()(1, 2);
		r22 = matrix.matrix()(2, 2);

		t0 = matrix.matrix()(0, 3);
		t1 = matrix.matrix()(1, 3);
		t2 = matrix.matrix()(2, 3);

		errCUDA = cudaTransformPointCloud(threads,
				this->d_second_point_cloud,
				second_point_cloud.points.size(),
				this->d_point_cloud,
				second_point_cloud.points.size(),
				r00, r10, r20, r01, r11, r21, r02, r12, r22, t0, t1, t2);
			throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

		int number_of_nn = 0 ;
		errCUDA = cudaCountNumberOfSemanticNearestNeighbours(
				this->threads,
				this->d_first_point_cloud,
				first_point_cloud.points.size(),
				this->d_point_cloud,
				second_point_cloud.points.size(),
				this->d_hashTable,
				this->d_buckets,
				rgd_params,
				search_radius,
				max_number_considered_in_INNER_bucket,
				max_number_considered_in_OUTER_bucket,
				this->d_nearest_neighbour_indexes,
				number_of_nn);
			throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

			if(number_of_nn > max_number_of_nn)
			{
				max_number_of_nn = number_of_nn;
				myaw = matrix;
				angle_res = i;
			}
			std::cout << "number_of_nn: " << number_of_nn << " for angle: "<< i <<  std::endl;
	}

	std::cout << "angle_result: " << angle_res << std::endl;

	errCUDA = cudaFree(this->d_first_point_cloud); this->d_first_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_second_point_cloud); this->d_second_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_point_cloud); this->d_point_cloud = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_hashTable); this->d_hashTable = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_buckets); this->d_buckets = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);

	errCUDA = cudaFree(this->d_nearest_neighbour_indexes); this->d_nearest_neighbour_indexes = 0;
		throw_on_cuda_error(errCUDA, __FILE__, __LINE__);


	return;
}
