#ifndef __CUDAWRAPPER__
#define __CUDAWRAPPER__

#include "lesson_16.h"
#include <pcl/point_cloud.h>
#include "custom_point_types.h"

#include "CCUDAAXBSolverWrapper.h"

#include <thrust/system_error.h>
#include <thrust/system/cuda/error.h>
#include <sstream>

typedef struct observations
{
	std::vector<obs_nn_t> vobs_nn;
	Eigen::Affine3f m_pose;
	double om;
	double fi;
	double ka;
	double tx;
	double ty;
	double tz;
}observations_t;

class CCudaWrapper
{
public:
	CCudaWrapper();
	~CCudaWrapper();

	void warmUpGPU(int cudaDevice);
	int getNumberOfAvailableThreads(int cudaDevice);
	bool getNumberOfAvailableThreads(int cudaDevice, int &threads, int &threadsNV);

	void coutMemoryStatus();

	//from lesson 2
	void removeNoiseNaive(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &point_cloud, float resolution, float bounding_box_extension, int number_of_points_in_bucket_threshold);

	//from lesson 1
	void downsampling(pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &point_cloud, float resolution, float bounding_box_extension);

	//from lesson 7
	void classify(  pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &point_cloud,
						float normal_vectors_search_radius,
						float curvature_threshold,
						float ground_Z_coordinate_threshold,
						int number_of_points_needed_for_plane_threshold,
						float bounding_box_extension,
						int max_number_considered_in_INNER_bucket,
						int max_number_considered_in_OUTER_bucket  );

	void semanticNearestNeighbourhoodSearch(
			pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &first_point_cloud,
			pcl::PointCloud<lidar_pointcloud::PointXYZIRNLRGB> &second_point_cloud,
			float search_radius,
			float bucket_size,
			float bounding_box_extension,
			int max_number_considered_in_INNER_bucket,
			int max_number_considered_in_OUTER_bucket,
			std::vector<int> &nearest_neighbour_indexes );

	void Matrix4ToEuler(const double *alignxf, double *rPosTheta, double *rPos);
	void Matrix4ToEuler(Eigen::Affine3f m, Eigen::Vector3f &omfika, Eigen::Vector3f &xyz);
	void EulerToMatrix(Eigen::Vector3f omfika, Eigen::Vector3f xyz, Eigen::Affine3f &m);

	bool registerLS(observations_t &obs);
	void throw_on_cuda_error(cudaError_t code, const char *file, int line);

	int threads;
	int threadsNV;

	lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud;
	lidar_pointcloud::PointXYZIRNLRGB *d_first_point_cloud;
	lidar_pointcloud::PointXYZIRNLRGB *d_second_point_cloud;
	int *d_nearest_neighbour_indexes;
	hashElement* d_hashTable;
	bucket* d_buckets;
	bool* d_markers;
	simple_point3D *d_mean;

	double *d_A;
	double *d_P;
	double *d_l;
	obs_nn_t *d_obs_nn;

};

/*
 * #include <thrust/system_error.h>
#include <thrust/system/cuda/error.h>
#include <sstream>

void throw_on_cuda_error(cudaError_t code, const char *file, int line)
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
 * */

#endif
