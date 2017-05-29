#ifndef __LESSON_16_H__
#define __LESSON_16_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <pcl/point_types.h>
#include "custom_point_types.h"

#define LABEL_PLANE 0
#define LABEL_EDGE 1
#define LABEL_CEILING 2
#define LABEL_GROUND 3


struct hashElement{
	int index_of_point;
	int index_of_bucket;
};

struct bucket{
	int index_begin;
	int index_end;
	int number_of_points;
};

struct gridParameters{
	float bounding_box_min_X;
	float bounding_box_min_Y;
	float bounding_box_min_Z;
	float bounding_box_max_X;
	float bounding_box_max_Y;
	float bounding_box_max_Z;
	int number_of_buckets_X;
	int number_of_buckets_Y;
	int number_of_buckets_Z;
	long long int number_of_buckets;
	float resolution_X;
	float resolution_Y;
	float resolution_Z;
};

struct simple_point3D{
	float x;
	float y;
	float z;
};

typedef struct obs_nn
{
	float x_diff;
	float y_diff;
	float z_diff;
	float x0;
	float y0;
	float z0;
	float P;
}obs_nn_t;

cudaError_t cudaWarmUpGPU();

cudaError_t cudaCalculateGridParams(lidar_pointcloud::PointXYZIRNLRGB* d_point_cloud, int number_of_points,
	float resolution_X, float resolution_Y, float resolution_Z, float bounding_box_extension, gridParameters &out_rgd_params);

cudaError_t cudaCalculateGrid(int threads, lidar_pointcloud::PointXYZIRNLRGB* d_point_cloud, bucket *d_buckets,
		hashElement *d_hashTable, int number_of_points, gridParameters rgd_params);

cudaError_t cudaCompute_AtP(int threads, double *d_A, double *d_P, double *d_AtP, int rows, int columns);

cudaError_t fill_A_l_cuda(int threads, double *d_A, double x, double y, double z, double om, double fi, double ka,
		obs_nn_t *d_obs_nn, int nop, double *d_P, double *d_l);

cudaError_t cudaSemanticNearestNeighborSearch(
		int threads,
		lidar_pointcloud::PointXYZIRNLRGB *d_first_point_cloud,
		int number_of_points_first_point_cloud,
		lidar_pointcloud::PointXYZIRNLRGB *d_second_point_cloud,
		int number_of_points_second_point_cloud,
		hashElement *d_hashTable,
		bucket *d_buckets,
		gridParameters rgd_params,
		float search_radius,
		int max_number_considered_in_INNER_bucket,
		int max_number_considered_in_OUTER_bucket,
		int *d_nearest_neighbour_indexes);

cudaError_t cudaRemoveNoiseNaive(int threads, bool *d_markers, lidar_pointcloud::PointXYZIRNLRGB* d_point_cloud,
		hashElement *d_hashTable, bucket *d_buckets, gridParameters rgd_params, int number_of_points, int number_of_points_in_bucket_threshold);

cudaError_t cudaDownSample(int threads, bool *d_markers, hashElement *d_hashTable, bucket *d_buckets, gridParameters rgd_params, int number_of_points);

cudaError_t cudaSemanticLabelingPlaneEdges(
		int threads,
		lidar_pointcloud::PointXYZIRNLRGB * d_point_cloud,
		int number_of_points,
		hashElement* d_hashTable,
		bucket* d_buckets,
		simple_point3D* d_mean,
		gridParameters rgd_params,
		float search_radius,
		int max_number_considered_in_INNER_bucket,
		int max_number_considered_in_OUTER_bucket,
		float curvature_threshold,
		int number_of_points_needed_for_plane_threshold);

cudaError_t cudaSemanticLabelingFloorCeiling(
		int threads,
		lidar_pointcloud::PointXYZIRNLRGB * d_point_cloud,
		int number_of_points,
		float ground_Z_coordinate_threshold);

#endif
