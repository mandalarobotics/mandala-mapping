#include "lesson_16.cuh"

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/sort.h>
#include <thrust/extrema.h>

#include "cuda_SVD.cu"

__global__ void kernel_cudaWarmUpGPU()
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	ind = ind + 1;
}

cudaError_t cudaWarmUpGPU()
{
	kernel_cudaWarmUpGPU<<<1,1>>>();
	cudaDeviceSynchronize();
	return cudaGetLastError();
}

cudaError_t cudaCalculateGridParams(lidar_pointcloud::PointXYZIRNLRGB* d_point_cloud, int number_of_points,
	float resolution_X, float resolution_Y, float resolution_Z, float bounding_box_extension, gridParameters &out_rgd_params)
{
	cudaError_t err = cudaGetLastError();

	try
	{
		thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB> t_cloud(d_point_cloud);
		err = cudaGetLastError();
		if(err != ::cudaSuccess)return err;
	
		thrust::pair<thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB>,thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB> >
		 minmaxX=thrust::minmax_element(t_cloud,t_cloud+number_of_points,compareX());
		err = cudaGetLastError();
		if(err != ::cudaSuccess)return err;
	
		thrust::pair<thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB>,thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB> >
		 minmaxY=thrust::minmax_element(t_cloud,t_cloud+number_of_points,compareY());
		err = cudaGetLastError();
		if(err != ::cudaSuccess)return err;
	
		thrust::pair<thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB>,thrust::device_ptr<lidar_pointcloud::PointXYZIRNLRGB> >
		 minmaxZ=thrust::minmax_element(t_cloud,t_cloud+number_of_points,compareZ());
		err = cudaGetLastError();
		if(err != ::cudaSuccess)return err;
		
		lidar_pointcloud::PointXYZIRNLRGB minX,maxX,minZ,maxZ,minY,maxY;

		err = cudaMemcpy(&minX,minmaxX.first.get(),sizeof(lidar_pointcloud::PointXYZIRNLRGB),cudaMemcpyDeviceToHost);
		if(err != ::cudaSuccess)return err;
		err = cudaMemcpy(&maxX,minmaxX.second.get(),sizeof(lidar_pointcloud::PointXYZIRNLRGB),cudaMemcpyDeviceToHost);
		if(err != ::cudaSuccess)return err;
		err = cudaMemcpy(&minZ,minmaxZ.first.get(),sizeof(lidar_pointcloud::PointXYZIRNLRGB),cudaMemcpyDeviceToHost);
		if(err != ::cudaSuccess)return err;
		err = cudaMemcpy(&maxZ,minmaxZ.second.get(),sizeof(lidar_pointcloud::PointXYZIRNLRGB),cudaMemcpyDeviceToHost);
		if(err != ::cudaSuccess)return err;
		err = cudaMemcpy(&minY,minmaxY.first.get(),sizeof(lidar_pointcloud::PointXYZIRNLRGB),cudaMemcpyDeviceToHost);
		if(err != ::cudaSuccess)return err;
		err = cudaMemcpy(&maxY,minmaxY.second.get(),sizeof(lidar_pointcloud::PointXYZIRNLRGB),cudaMemcpyDeviceToHost);
		if(err != ::cudaSuccess)return err;
	
		maxX.x += bounding_box_extension;
		minX.x -= bounding_box_extension;

		maxY.y += bounding_box_extension;
		minY.y -= bounding_box_extension;

		maxZ.z += bounding_box_extension;
		minZ.z -= bounding_box_extension;

		int number_of_buckets_X=((maxX.x-minX.x)/resolution_X)+1;
		int number_of_buckets_Y=((maxY.y-minY.y)/resolution_Y)+1;
		int number_of_buckets_Z=((maxZ.z-minZ.z)/resolution_Z)+1;

		out_rgd_params.number_of_buckets_X=number_of_buckets_X;
		out_rgd_params.number_of_buckets_Y=number_of_buckets_Y;
		out_rgd_params.number_of_buckets_Z=number_of_buckets_Z;
		out_rgd_params.number_of_buckets = number_of_buckets_X * number_of_buckets_Y * number_of_buckets_Z;
		
		out_rgd_params.bounding_box_max_X=maxX.x;
		out_rgd_params.bounding_box_min_X=minX.x;
		out_rgd_params.bounding_box_max_Y=maxY.y;
		out_rgd_params.bounding_box_min_Y=minY.y;
		out_rgd_params.bounding_box_max_Z=maxZ.z;
		out_rgd_params.bounding_box_min_Z=minZ.z;
		
		out_rgd_params.resolution_X=resolution_X;
		out_rgd_params.resolution_Y=resolution_Y;
		out_rgd_params.resolution_Z=resolution_Z;
	}
	catch(thrust::system_error &e)
	{
		err = cudaGetLastError();
		cudaDeviceReset();
		return err;
	}	
	catch(std::bad_alloc &e)
  	{
  	 	err = cudaGetLastError();
		cudaDeviceReset();
		return err;
  	}	
	return cudaGetLastError();
}


__global__ void kernel_initializeIndByKey(hashElement* d_hashTable, int number_of_points)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < number_of_points)
	{
		d_hashTable[ind].index_of_point=ind;
		d_hashTable[ind].index_of_bucket=0;
	}
}

__global__ void kernel_getIndexOfBucketForPoints(lidar_pointcloud::PointXYZIRNLRGB* cloud, hashElement* d_hashTable, int number_of_points, gridParameters rgd_params)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < number_of_points)
	{
		int ix=(cloud[ind].x-rgd_params.bounding_box_min_X)/rgd_params.resolution_X;
		int iy=(cloud[ind].y-rgd_params.bounding_box_min_Y)/rgd_params.resolution_Y;
		int iz=(cloud[ind].z-rgd_params.bounding_box_min_Z)/rgd_params.resolution_Z;
		d_hashTable[ind].index_of_bucket=ix*rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z+iy*rgd_params.number_of_buckets_Z+iz;
	}
}

__global__ void kernel_initializeBuckets(bucket* d_buckets, gridParameters rgd_params)
{
	long long int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < rgd_params.number_of_buckets)
	{
		d_buckets[ind].index_begin=-1;
		d_buckets[ind].index_end=-1;
		d_buckets[ind].number_of_points=0;
	}
}

__global__ void kernel_updateBuckets(hashElement* d_hashTable, bucket* d_buckets,
		gridParameters rgd_params, int number_of_points)
{
	int ind = blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < number_of_points)
	{
		if(ind == 0)
		{
			int index_of_bucket = d_hashTable[ind].index_of_bucket;
			int index_of_bucket_1 = d_hashTable[ind+1].index_of_bucket;

			d_buckets[index_of_bucket].index_begin=ind;
			if(index_of_bucket != index_of_bucket_1)
			{
				d_buckets[index_of_bucket].index_end=ind+1;
				d_buckets[index_of_bucket_1].index_end=ind+1;
			}
		}else if(ind == number_of_points-1)
		{
			d_buckets[d_hashTable[ind].index_of_bucket].index_end=ind+1;
		}else
		{
			int index_of_bucket = d_hashTable[ind].index_of_bucket;
			int index_of_bucket_1 = d_hashTable[ind+1].index_of_bucket;

			if(index_of_bucket != index_of_bucket_1)
			{
				d_buckets[index_of_bucket].index_end=ind+1;
				d_buckets[index_of_bucket_1].index_begin=ind+1;
			}
		}
	}
}

__global__ void kernel_countNumberOfPointsForBuckets(bucket* d_buckets, gridParameters rgd_params)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < rgd_params.number_of_buckets)
	{
		int index_begin = d_buckets[ind].index_begin;
		int index_end = d_buckets[ind].index_end;

		if(index_begin != -1 && index_end !=-1)
		{
			d_buckets[ind].number_of_points = index_end - index_begin;
		}
	}
}

__global__ void kernel_copyKeys(hashElement* d_hashTable_in, hashElement* d_hashTable_out, int number_of_points)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < number_of_points)
	{
		d_hashTable_out[ind] = d_hashTable_in[ind];
	}
}

cudaError_t cudaCalculateGrid(int threads, lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud, bucket *d_buckets,
		hashElement *d_hashTable, int number_of_points, gridParameters rgd_params)
{
	cudaError_t err = cudaGetLastError();
	hashElement* d_temp_hashTable;	cudaMalloc((void**)&d_temp_hashTable,number_of_points*sizeof(hashElement));
	int blocks=number_of_points/threads + 1;

	kernel_initializeIndByKey<<<blocks,threads>>>(d_temp_hashTable, number_of_points);
	err = cudaDeviceSynchronize();	if(err != ::cudaSuccess)return err;

	kernel_getIndexOfBucketForPoints<<<blocks,threads>>>(d_point_cloud, d_temp_hashTable, number_of_points, rgd_params);
	err = cudaDeviceSynchronize();	if(err != ::cudaSuccess)return err;

	try
	{
		thrust::device_ptr<hashElement> t_d_temp_hashTable(d_temp_hashTable);
		thrust::sort(t_d_temp_hashTable,t_d_temp_hashTable+number_of_points,compareHashElements());
	}
	catch(thrust::system_error &e)
	{
		err = cudaGetLastError();
		return err;
	}
	catch(std::bad_alloc &e)
	{
		err = cudaGetLastError();
		return err;
	}

	kernel_initializeBuckets<<<rgd_params.number_of_buckets/threads+1,threads>>>(d_buckets,rgd_params);
	err = cudaDeviceSynchronize();	if(err != ::cudaSuccess)return err;

	kernel_updateBuckets<<<blocks,threads>>>(d_temp_hashTable, d_buckets, rgd_params, number_of_points);
	err = cudaDeviceSynchronize();	if(err != ::cudaSuccess)return err;

	kernel_countNumberOfPointsForBuckets<<<rgd_params.number_of_buckets/threads+1,threads>>>(d_buckets, rgd_params);
	err = cudaDeviceSynchronize();	if(err != ::cudaSuccess)return err;

	kernel_copyKeys<<<blocks,threads>>>(d_temp_hashTable, d_hashTable, number_of_points);
	err = cudaDeviceSynchronize(); if(err != ::cudaSuccess)return err;

	err = cudaFree(d_temp_hashTable);
	return err;
}

__global__ void kernel_cudaCompute_AtP(double *d_A, double *d_P, double *d_AtP, int rows, int columns )
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind<rows*columns)
	{
		int row = ind%rows;
		int column = ind/rows;

		d_AtP[row + column * rows] = d_A[column + row * columns] * d_P[column];
	}
}

cudaError_t cudaCompute_AtP(int threads, double *d_A, double *d_P, double *d_AtP, int rows, int columns)
{
	cudaError_t err = ::cudaSuccess;

	kernel_cudaCompute_AtP<<<(rows*columns)/threads+1,threads>>>(d_A, d_P, d_AtP, rows, columns);

	err = cudaDeviceSynchronize();
	return err;
}


#define _11 0
#define _12 1
#define _13 2
#define _21 3
#define _22 4
#define _23 5
#define _31 6
#define _32 7
#define _33 8

__device__ void computeR(double om, double fi, double ka, double *R)
{
	//R[11 12 13; 21 22 23; 31 32 33]
	//R[0  1  2 ; 3  4  5 ; 6  7  8]
	R[_11] = cos(fi) * cos(ka);
	R[_12] = -cos(fi) * sin(ka);
	R[_13] = sin(fi);

	R[_21] = cos(om)*sin(ka) + sin(om)*sin(fi)*cos(ka);
	R[_22] = cos(om) *cos(ka) - sin(om)*sin(fi)*sin(ka);
	R[_23] = -sin(om) * cos(fi);

	R[_31] = sin(om) * sin(ka) - cos(om)*sin(fi)*cos(ka);
	R[_32] = sin(om) * cos(ka) + cos(om)*sin(fi)*sin(ka);
	R[_33] = cos(om) * cos(fi);
}

__device__ double compute_a10(double *r, double x0, double y0, double z0)
{
	return r[_11]*x0 + r[_12] * y0 + r[_13] * z0;
}

__device__ double compute_a20(double *r, double x0, double y0, double z0)
{
	return r[_21]*x0 + r[_22] * y0 + r[_23] * z0;
}

__device__ double compute_a30(double *r, double x0, double y0, double z0)
{
	return r[_31] * x0 + r[_32] * y0 + r[_33]*z0;
}

__device__ double compute_a11()
{
	return 0.0;
}

__device__ double compute_a12(double m, double om, double fi, double ka, double x0, double y0, double z0)
{
	return m*(-sin(fi)*cos(ka)*x0 + sin(fi)*sin(ka)*y0 + cos (fi) *z0);
}

__device__ double compute_a13(double m, double *r, double x0, double y0)
{
	return m*(r[_12]*x0-r[_11]*y0);
}

__device__ double compute_a21(double m, double *r, double x0, double y0, double z0)
{
	return m*(-r[_31]*x0-r[_32]*y0-r[_33]*z0);
}

__device__ double compute_a22(double m, double om, double fi, double ka, double x0, double y0, double z0)
{
	return m*(sin(om)*cos(fi)*cos(ka)*x0 - sin(om)*cos(fi)*sin(ka)*y0+sin(om)*sin(fi)*z0);
}

__device__ double compute_a23(double m, double *r, double x0, double y0)
{
	return m*(r[_22]*x0-r[_21]*y0);
}

__device__ double compute_a31(double m, double *r, double x0, double y0, double z0)
{
	return m*(r[_21]*x0+r[_22]*y0 +r[_23]*z0);
}

__device__ double compute_a32(double m, double om, double fi, double ka, double x0, double y0, double z0)
{
	return m * (-cos(om)*cos(fi)*cos(ka)*x0 + cos(om)*cos(fi)*sin(ka)*y0 - cos(om)*sin(fi)*z0);
}

__device__ double compute_a33(double m, double *r, double x0, double y0)
{
	return m*(r[_32]*x0 - r[_31]*y0);
}

__global__ void  kernel_fill_A_l_cuda(double *d_A, double x, double y, double z, double om, double fi, double ka,
		obs_nn_t *d_obs_nn, int nop, double *d_P, double *d_l)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;

	if(ind<nop)
	{
		obs_nn_t obs_nn = d_obs_nn[ind];
		double r[9];
		computeR(om, fi, ka, r);
		double x0 = obs_nn.x0;
		double y0 = obs_nn.y0;
		double z0 = obs_nn.z0;

		double a11 = -1.0;
		double a12 =  0.0;
		double a13 =  0.0;

		double a21 =  0.0;
		double a22 = -1.0;
		double a23 =  0.0;

		double a31 =  0.0;
		double a32 =  0.0;
		double a33 = -1.0;

		double m = 1.0;

		double a14 = compute_a11();
		double a15 = compute_a12(m, om, fi, ka, x0, y0, z0);
		double a16 = compute_a13(m, r, x0, y0);

		double a24 = compute_a21(m, r, x0, y0, z0);
		double a25 = compute_a22(m, om, fi, ka, x0, y0, z0);
		double a26 = compute_a23(m, r, x0, y0);

		double a34 = compute_a31(m, r, x0, y0, z0);
		double a35 = compute_a32(m, om, fi, ka, x0, y0, z0);
		double a36 = compute_a33(m, r, x0, y0);

		d_A[ind * 3 + 0 + 0 * nop * 3] = a11;
		d_A[ind * 3 + 1 + 0 * nop * 3] = a21;
		d_A[ind * 3 + 2 + 0 * nop * 3] = a31;

		d_A[ind * 3 + 0 + 1 * nop * 3] = a12;
		d_A[ind * 3 + 1 + 1 * nop * 3] = a22;
		d_A[ind * 3 + 2 + 1 * nop * 3] = a32;

		d_A[ind * 3 + 0 + 2 * nop * 3] = a13;
		d_A[ind * 3 + 1 + 2 * nop * 3] = a23;
		d_A[ind * 3 + 2 + 2 * nop * 3] = a33;

		d_A[ind * 3 + 0 + 3 * nop * 3] = -a14;
		d_A[ind * 3 + 1 + 3 * nop * 3] = -a24;
		d_A[ind * 3 + 2 + 3 * nop * 3] = -a34;

		d_A[ind * 3 + 0 + 4 * nop * 3] = -a15;
		d_A[ind * 3 + 1 + 4 * nop * 3] = -a25;
		d_A[ind * 3 + 2 + 4 * nop * 3] = -a35;

		d_A[ind * 3 + 0 + 5 * nop * 3] = -a16;
		d_A[ind * 3 + 1 + 5 * nop * 3] = -a26;
		d_A[ind * 3 + 2 + 5 * nop * 3] = -a36;

		d_P[ind * 3    ] = obs_nn.P;
		d_P[ind * 3 + 1] = obs_nn.P;
		d_P[ind * 3 + 2] = obs_nn.P;

		d_l[ind * 3    ] = obs_nn.x_diff;
		d_l[ind * 3 + 1] = obs_nn.y_diff;
		d_l[ind * 3 + 2] = obs_nn.z_diff;

	}
}

cudaError_t fill_A_l_cuda(int threads, double *d_A, double x, double y, double z, double om, double fi, double ka,
		obs_nn_t *d_obs_nn, int nop, double *d_P, double *d_l)
{
	cudaError_t err = ::cudaSuccess;
	int blocks=nop/threads+1;
	kernel_fill_A_l_cuda<<<blocks,threads>>>(d_A, x, y, z, om, fi, ka, d_obs_nn, nop, d_P, d_l);

	err = cudaDeviceSynchronize();
	return err;
}

__global__ void  kernel_fill_A_l_4DOFcuda(double *d_A, double x, double y, double z, double om, double fi, double ka,
		obs_nn_t *d_obs_nn, int nop, double *d_P, double *d_l)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;

	if(ind<nop)
	{
		obs_nn_t obs_nn = d_obs_nn[ind];
		double r[9];
		computeR(om, fi, ka, r);
		double x0 = obs_nn.x0;
		double y0 = obs_nn.y0;
		double z0 = obs_nn.z0;

		double a11 = -1.0;
		double a12 =  0.0;
		double a13 =  0.0;

		double a21 =  0.0;
		double a22 = -1.0;
		double a23 =  0.0;

		double a31 =  0.0;
		double a32 =  0.0;
		double a33 = -1.0;

		double m = 1.0;

		double a14 = compute_a11();
		double a15 = compute_a12(m, om, fi, ka, x0, y0, z0);
		double a16 = compute_a13(m, r, x0, y0);

		double a24 = compute_a21(m, r, x0, y0, z0);
		double a25 = compute_a22(m, om, fi, ka, x0, y0, z0);
		double a26 = compute_a23(m, r, x0, y0);

		double a34 = compute_a31(m, r, x0, y0, z0);
		double a35 = compute_a32(m, om, fi, ka, x0, y0, z0);
		double a36 = compute_a33(m, r, x0, y0);

		d_A[ind * 3 + 0 + 0 * nop * 3] = a11;
		d_A[ind * 3 + 1 + 0 * nop * 3] = a21;
		d_A[ind * 3 + 2 + 0 * nop * 3] = a31;

		d_A[ind * 3 + 0 + 1 * nop * 3] = a12;
		d_A[ind * 3 + 1 + 1 * nop * 3] = a22;
		d_A[ind * 3 + 2 + 1 * nop * 3] = a32;

		d_A[ind * 3 + 0 + 2 * nop * 3] = a13;
		d_A[ind * 3 + 1 + 2 * nop * 3] = a23;
		d_A[ind * 3 + 2 + 2 * nop * 3] = a33;

		d_A[ind * 3 + 0 + 3 * nop * 3] = -a16;
		d_A[ind * 3 + 1 + 3 * nop * 3] = -a26;
		d_A[ind * 3 + 2 + 3 * nop * 3] = -a36;

		/*d_A[ind * 3 + 0 + 3 * nop * 3] = -a14;
		d_A[ind * 3 + 1 + 3 * nop * 3] = -a24;
		d_A[ind * 3 + 2 + 3 * nop * 3] = -a34;

		d_A[ind * 3 + 0 + 4 * nop * 3] = -a15;
		d_A[ind * 3 + 1 + 4 * nop * 3] = -a25;
		d_A[ind * 3 + 2 + 4 * nop * 3] = -a35;

		d_A[ind * 3 + 0 + 5 * nop * 3] = -a16;
		d_A[ind * 3 + 1 + 5 * nop * 3] = -a26;
		d_A[ind * 3 + 2 + 5 * nop * 3] = -a36;*/

		d_P[ind * 3    ] = obs_nn.P;
		d_P[ind * 3 + 1] = obs_nn.P;
		d_P[ind * 3 + 2] = obs_nn.P;

		d_l[ind * 3    ] = obs_nn.x_diff;
		d_l[ind * 3 + 1] = obs_nn.y_diff;
		d_l[ind * 3 + 2] = obs_nn.z_diff;

	}
}

cudaError_t fill_A_l_4DOFcuda(int threads, double *d_A, double x, double y, double z, double om, double fi, double ka,
		obs_nn_t *d_obs_nn, int nop, double *d_P, double *d_l)
{
	cudaError_t err = ::cudaSuccess;
	int blocks=nop/threads+1;
	kernel_fill_A_l_4DOFcuda<<<blocks,threads>>>(d_A, x, y, z, om, fi, ka, d_obs_nn, nop, d_P, d_l);

	err = cudaDeviceSynchronize();
	return err;
}

__global__ void kernel_semanticNearestNeighborSearch(
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
		int *d_nearest_neighbour_indexes)
{
	int index_of_point_second_point_cloud = blockIdx.x*blockDim.x+threadIdx.x;

	if(index_of_point_second_point_cloud < number_of_points_second_point_cloud)
	{
		bool isok = false;

		float x = d_second_point_cloud[index_of_point_second_point_cloud].x;
		float y = d_second_point_cloud[index_of_point_second_point_cloud].y;
		float z = d_second_point_cloud[index_of_point_second_point_cloud].z;
		int label_second_point_cloud =  d_second_point_cloud[index_of_point_second_point_cloud].label;

		if(x < rgd_params.bounding_box_min_X || x > rgd_params.bounding_box_max_X)
		{
			d_nearest_neighbour_indexes[index_of_point_second_point_cloud] = -1;
			return;
		}
		if(y < rgd_params.bounding_box_min_Y || y > rgd_params.bounding_box_max_Y)
		{
			d_nearest_neighbour_indexes[index_of_point_second_point_cloud] = -1;
			return;
		}
		if(z < rgd_params.bounding_box_min_Z || z > rgd_params.bounding_box_max_Z)
		{
			d_nearest_neighbour_indexes[index_of_point_second_point_cloud] = -1;
			return;
		}

		int ix=(x - rgd_params.bounding_box_min_X)/rgd_params.resolution_X;
		int iy=(y - rgd_params.bounding_box_min_Y)/rgd_params.resolution_Y;
		int iz=(z - rgd_params.bounding_box_min_Z)/rgd_params.resolution_Z;

		int index_bucket = ix*rgd_params.number_of_buckets_Y *
				rgd_params.number_of_buckets_Z + iy * rgd_params.number_of_buckets_Z + iz;
		int nn_index = -1;

		if(index_bucket >= 0 && index_bucket < rgd_params.number_of_buckets)
		{
			int sx, sy, sz, stx, sty, stz;
			if(ix == 0)sx = 0; else sx = -1;
			if(iy == 0)sy = 0; else sy = -1;
			if(iz == 0)sz = 0; else sz =- 1;

			if(ix == rgd_params.number_of_buckets_X - 1)stx = 1; else stx = 2;
			if(iy == rgd_params.number_of_buckets_Y - 1)sty = 1; else sty = 2;
			if(iz == rgd_params.number_of_buckets_Z - 1)stz = 1; else stz = 2;

			float _distance = 100000000.0f;
			int index_next_bucket;
			int iter;
			int number_of_points_in_bucket;
			int l_begin;
			int l_end;

			for(int i = sx; i < stx; i++)
			{
				for(int j = sy; j < sty; j++)
				{
					for(int k = sz; k < stz; k++)
					{
						index_next_bucket = index_bucket +
								i * rgd_params.number_of_buckets_Y * rgd_params.number_of_buckets_Z +
								j * rgd_params.number_of_buckets_Z + k;
						if(index_next_bucket >= 0 && index_next_bucket < rgd_params.number_of_buckets)
						{
							number_of_points_in_bucket = d_buckets[index_next_bucket].number_of_points;
							if(number_of_points_in_bucket <= 0)continue;

							int max_number_considered_in_bucket;
							if(index_next_bucket == index_bucket)
							{
								max_number_considered_in_bucket = max_number_considered_in_INNER_bucket;
							}else
							{
								max_number_considered_in_bucket = max_number_considered_in_OUTER_bucket;
							}
							if(max_number_considered_in_bucket <= 0)continue;

							if(max_number_considered_in_bucket >= number_of_points_in_bucket)
							{
								iter=1;
							}else
							{
								iter = number_of_points_in_bucket / max_number_considered_in_bucket;
								if(iter <= 0)iter = 1;
							}

							l_begin = d_buckets[index_next_bucket].index_begin;
							l_end = d_buckets[index_next_bucket].index_end;

							for(int l = l_begin; l < l_end; l += iter)
							{
								if(l >= 0 && l < number_of_points_first_point_cloud)
								{
									int hashed_index_of_point = d_hashTable[l].index_of_point;
									//inA[hashed_index_of_point].var = 1;

									float nn_x  = d_first_point_cloud[hashed_index_of_point].x;
									float nn_y  = d_first_point_cloud[hashed_index_of_point].y;
									float nn_z  = d_first_point_cloud[hashed_index_of_point].z;

									int label_first_point_cloud = d_first_point_cloud[hashed_index_of_point].label;

									float dist  = (x - nn_x) * (x - nn_x) +
												  (y - nn_y) * (y - nn_y) +
												  (z - nn_z) * (z - nn_z);

									if(label_first_point_cloud == label_second_point_cloud)
									{
										if(dist <= search_radius * search_radius )
										{
											if(dist < _distance)
											{
												isok = true;
												nn_index = hashed_index_of_point;
												_distance = dist;
											}
										}
									}
								}
							}
						}
					}
				}
			}
		}

		if(isok)
		{
			d_nearest_neighbour_indexes[index_of_point_second_point_cloud] = nn_index;

		}else
		{
			d_nearest_neighbour_indexes[index_of_point_second_point_cloud] = -1;
		}
	}
}

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
			int *d_nearest_neighbour_indexes)
{
	cudaError_t err = cudaGetLastError();

	int blocks=number_of_points_second_point_cloud/threads+1;

	kernel_semanticNearestNeighborSearch<<<blocks,threads>>>(
			d_first_point_cloud,
			number_of_points_first_point_cloud,
			d_second_point_cloud,
			number_of_points_second_point_cloud,
			d_hashTable,
			d_buckets,
			rgd_params,
			search_radius,
			max_number_considered_in_INNER_bucket,
			max_number_considered_in_OUTER_bucket,
			d_nearest_neighbour_indexes);

	err = cudaDeviceSynchronize();
	return err;
}

__global__ void kernel_setAllPointsToRemove(bool *d_markers, int number_of_points)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind<number_of_points)
	{
		d_markers[ind] = false;
	}
}

__global__ void kernel_markPointsToRemain(bool *d_markers, lidar_pointcloud::PointXYZIRNLRGB* d_point_cloud, hashElement *d_hashTable, bucket *d_buckets,
 gridParameters rgd_params, int number_of_points, int number_of_points_in_bucket_threshold)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind < number_of_points)
	{
		int ix=(d_point_cloud[ind].x-rgd_params.bounding_box_min_X)/rgd_params.resolution_X;
		int iy=(d_point_cloud[ind].y-rgd_params.bounding_box_min_Y)/rgd_params.resolution_Y;
		int iz=(d_point_cloud[ind].z-rgd_params.bounding_box_min_Z)/rgd_params.resolution_Z;
		int index_of_bucket=ix*rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z+iy*rgd_params.number_of_buckets_Z+iz;

		if(index_of_bucket >=0 && index_of_bucket < rgd_params.number_of_buckets)
		{
			bucket temp_bucket = d_buckets[index_of_bucket];
			if(temp_bucket.number_of_points > number_of_points_in_bucket_threshold)d_markers[ind] = true;
		}
	}
}

cudaError_t cudaRemoveNoiseNaive(int threads, bool *d_markers, lidar_pointcloud::PointXYZIRNLRGB* d_point_cloud,
		hashElement *d_hashTable, bucket *d_buckets, gridParameters rgd_params, int number_of_points,
		int number_of_points_in_bucket_threshold)
{
	cudaError_t err = cudaGetLastError();

	kernel_setAllPointsToRemove<<<number_of_points/threads+1,threads>>>(d_markers, number_of_points);
	err = cudaDeviceSynchronize();	if(err != ::cudaSuccess)return err;

	kernel_markPointsToRemain<<<number_of_points/threads+1,threads>>>
			(d_markers, d_point_cloud, d_hashTable, d_buckets, rgd_params, number_of_points, number_of_points_in_bucket_threshold);

	err = cudaDeviceSynchronize();
	return err;
}

__global__ void kernel_markFirstPointInBuckets(bool *d_markers, hashElement *d_hashTable, bucket *d_buckets,
 gridParameters rgd_params)
{
	int ind=blockIdx.x*blockDim.x+threadIdx.x;
	if(ind<rgd_params.number_of_buckets)
	{
		int index_of_point = d_buckets[ind].index_begin;
		if(index_of_point != -1)
		{
			d_markers[d_hashTable[index_of_point].index_of_point] = true;
		}
	}
}

cudaError_t cudaDownSample(int threads, bool *d_markers, hashElement *d_hashTable, bucket *d_buckets, gridParameters rgd_params, int number_of_points)
{
	cudaError_t err = cudaGetLastError();
	kernel_setAllPointsToRemove<<<number_of_points/threads+1,threads>>>(d_markers, number_of_points);

		err = cudaDeviceSynchronize(); if(err != ::cudaSuccess)return err;

	kernel_markFirstPointInBuckets<<<rgd_params.number_of_buckets/threads+1,threads>>>
		(d_markers, d_hashTable, d_buckets, rgd_params);

		err = cudaDeviceSynchronize();

	return err;
}

__global__ void kernel_normalvectorcomputation_step1_fast(
		lidar_pointcloud::PointXYZIRNLRGB * d_point_cloud,
		hashElement* d_hashTable,
		simple_point3D* d_mean,
		int number_of_points,
		bucket* d_buckets,
		gridParameters rgd_params,
		float search_radius,
		int max_number_considered_in_INNER_bucket,
		int max_number_considered_in_OUTER_bucket)
{
	int index_of_point = blockIdx.x * blockDim.x + threadIdx.x;

	if(index_of_point < number_of_points)
	{
		d_point_cloud[index_of_point].normal_x = 0;
		d_point_cloud[index_of_point].normal_y = 0;
		d_point_cloud[index_of_point].normal_z = 0;
		__syncthreads();

		int index_of_bucket = d_hashTable[index_of_point].index_of_bucket;

		if(index_of_bucket >= 0 && index_of_bucket < rgd_params.number_of_buckets)
		{
			int hashed_index_of_point = d_hashTable[index_of_point].index_of_point;

			if(hashed_index_of_point >= 0 && hashed_index_of_point < number_of_points)
			{
				float x = d_point_cloud[hashed_index_of_point].x;
				float y = d_point_cloud[hashed_index_of_point].y;
				float z = d_point_cloud[hashed_index_of_point].z;

				int ix = index_of_bucket/(rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z);
				int iy = (index_of_bucket%(rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z))/rgd_params.number_of_buckets_Z;
				int iz = (index_of_bucket%(rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z))%rgd_params.number_of_buckets_Z;

				int sx, sy, sz, stx, sty, stz;
				if(ix == 0) sx = 0; else sx = -1;
				if(iy == 0) sy = 0; else sy = -1;
				if(iz == 0) sz = 0; else sz = -1;

				if(ix == rgd_params.number_of_buckets_X - 1)stx = 1; else stx = 2;
				if(iy == rgd_params.number_of_buckets_Y - 1)sty = 1; else sty = 2;
				if(iz == rgd_params.number_of_buckets_Z - 1)stz = 1; else stz = 2;

				int number_of_nearest_neighbours = 0;
				simple_point3D mean;
				mean.x = 0.0f;
				mean.y = 0.0f;
				mean.z = 0.0f;

				float nearest_neighbour_x;
				float nearest_neighbour_y;
				float nearest_neighbour_z;

				for(int i = sx; i < stx; i++)
				{
					for(int j = sy; j < sty; j++)
					{
						for(int k = sz; k < stz; k++)
						{
							int index_of_neighbour_bucket=index_of_bucket+i*rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z+j*rgd_params.number_of_buckets_Z+k;

							if(index_of_neighbour_bucket >= 0 && index_of_neighbour_bucket < rgd_params.number_of_buckets)
							{
								int iter;
								int number_of_points_in_bucket = d_buckets[index_of_neighbour_bucket].number_of_points;
								if(number_of_points_in_bucket <= 0)continue;

								int max_number_considered_in_bucket;
								if(index_of_neighbour_bucket==index_of_bucket)
								{
									max_number_considered_in_bucket = max_number_considered_in_INNER_bucket;
								}else
								{
									max_number_considered_in_bucket = max_number_considered_in_OUTER_bucket;
								}
								if(max_number_considered_in_bucket <= 0)continue;

								if(max_number_considered_in_bucket >= number_of_points_in_bucket)
								{
									iter=1;
								}else
								{
									iter = number_of_points_in_bucket / max_number_considered_in_bucket;
									if(iter <= 0)iter = 1;
								}

								int l_begin = d_buckets[index_of_neighbour_bucket].index_begin;
								int l_end = d_buckets[index_of_neighbour_bucket].index_end;

								for(int l = l_begin; l < l_end; l += iter)
								{
									if(l >= 0 && l < number_of_points)
									{
										int indexNextPointInBucket = d_hashTable[l].index_of_point;
										nearest_neighbour_x = d_point_cloud[indexNextPointInBucket].x;
										nearest_neighbour_y = d_point_cloud[indexNextPointInBucket].y;
										nearest_neighbour_z = d_point_cloud[indexNextPointInBucket].z;

										float dist=sqrtf((x - nearest_neighbour_x)*(x - nearest_neighbour_x)
														+(y - nearest_neighbour_y)*(y - nearest_neighbour_y)
														+(z - nearest_neighbour_z)*(z - nearest_neighbour_z));

										if(dist <= search_radius)
										{
											mean.x += nearest_neighbour_x;
											mean.y += nearest_neighbour_y;
											mean.z += nearest_neighbour_z;
											number_of_nearest_neighbours++;
										}
									}
								}
							}
						}
					}
				}

				if(number_of_nearest_neighbours >= 3)
				{
					d_mean[index_of_point].x = mean.x / number_of_nearest_neighbours;
					d_mean[index_of_point].y = mean.y / number_of_nearest_neighbours;
					d_mean[index_of_point].z = mean.z / number_of_nearest_neighbours;
				}else
				{
					d_mean[index_of_point].x = 0.0f;
					d_mean[index_of_point].y = 0.0f;
					d_mean[index_of_point].z = 0.0f;
				}
			}
		}
	}
}

__global__ void kernel_normalvectorcomputation_step2_fast_with_classification(
	lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud,
	hashElement *d_hashTable,
	simple_point3D *d_mean,
	int number_of_points,
	bucket *d_buckets,
	gridParameters rgd_params,
	float search_radius,
	int max_number_considered_in_INNER_bucket,
	int max_number_considered_in_OUTER_bucket,
	float curvature_threshold,
	int number_of_points_needed_for_plane_threshold)
{
	int index_of_point = blockIdx.x * blockDim.x + threadIdx.x;
	if(index_of_point < number_of_points)
	{
		int index_of_bucket = d_hashTable[index_of_point].index_of_bucket;
		if(index_of_bucket >= 0 && index_of_bucket < rgd_params.number_of_buckets)
		{
			int hashed_index_of_point = d_hashTable[index_of_point].index_of_point;
			if(hashed_index_of_point >= 0 && hashed_index_of_point < number_of_points)
			{
				d_point_cloud[hashed_index_of_point].label = LABEL_EDGE;

				simple_point3D mean = d_mean[index_of_point];
				if(mean.x != 0.0f && mean.y != 0.0f && mean.z != 0.0f)
				{
					float x = d_point_cloud[hashed_index_of_point].x;
					float y = d_point_cloud[hashed_index_of_point].y;
					float z = d_point_cloud[hashed_index_of_point].z;

					int ix = index_of_bucket/(rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z);
					int iy = (index_of_bucket%(rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z))/rgd_params.number_of_buckets_Z;
					int iz = (index_of_bucket%(rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z))%rgd_params.number_of_buckets_Z;
					int sx, sy, sz, stx, sty, stz;
					if(ix == 0)sx = 0; else sx = -1;
					if(iy == 0)sy = 0; else sy = -1;
					if(iz == 0)sz = 0; else sz = -1;
					if(ix == rgd_params.number_of_buckets_X - 1)stx = 1; else stx = 2;
					if(iy == rgd_params.number_of_buckets_Y - 1)sty = 1; else sty = 2;
					if(iz == rgd_params.number_of_buckets_Z - 1)stz = 1; else stz = 2;

					int number_of_nearest_neighbours=0;

					double cov[3][3];
					cov[0][0]=cov[0][1]=cov[0][2]=cov[1][0]=cov[1][1]=cov[1][2]=cov[2][0]=cov[2][1]=cov[2][2]=0;

					float nearest_neighbour_x;
					float nearest_neighbour_y;
					float nearest_neighbour_z;

					for(int i = sx; i < stx; i++)
					{
						for(int j = sy; j < sty; j++)
						{
							for(int k = sz; k < stz; k++)
							{
								int index_of_neighbour_bucket=index_of_bucket+i*rgd_params.number_of_buckets_Y*rgd_params.number_of_buckets_Z+j*rgd_params.number_of_buckets_Z+k;
								if(index_of_neighbour_bucket >= 0 && index_of_neighbour_bucket < rgd_params.number_of_buckets)
								{
									int iter;
									int number_of_points_in_bucket = d_buckets[index_of_neighbour_bucket].number_of_points;
									if(number_of_points_in_bucket <= 0)continue;

									int max_number_considered_in_bucket;
									if(index_of_neighbour_bucket==index_of_bucket)
									{
										max_number_considered_in_bucket = max_number_considered_in_INNER_bucket;
									}else
									{
										max_number_considered_in_bucket = max_number_considered_in_OUTER_bucket;
									}
									if(max_number_considered_in_bucket <= 0)continue;


									if(max_number_considered_in_bucket >= number_of_points_in_bucket)
									{
										iter=1;
									}else
									{
										iter = number_of_points_in_bucket / max_number_considered_in_bucket;
										if(iter <= 0)iter = 1;
									}

									int l_begin = d_buckets[index_of_neighbour_bucket].index_begin;
									int l_end = d_buckets[index_of_neighbour_bucket].index_end;

									for(int l = l_begin; l < l_end; l += iter)
									{
										if(l >= 0 && l < number_of_points)
										{
											int indexNextPointInBucket = d_hashTable[l].index_of_point;
											nearest_neighbour_x = d_point_cloud[indexNextPointInBucket].x;
											nearest_neighbour_y = d_point_cloud[indexNextPointInBucket].y;
											nearest_neighbour_z = d_point_cloud[indexNextPointInBucket].z;

											float dist = sqrtf((x - nearest_neighbour_x)*(x - nearest_neighbour_x)
															  +(y - nearest_neighbour_y)*(y - nearest_neighbour_y)
   															  +(z - nearest_neighbour_z)*(z - nearest_neighbour_z));

											if(dist <= search_radius)
											{
												cov[0][0]+=(mean.x - nearest_neighbour_x) * (mean.x - nearest_neighbour_x);
												cov[0][1]+=(mean.x - nearest_neighbour_x) * (mean.y - nearest_neighbour_y);
												cov[0][2]+=(mean.x - nearest_neighbour_x) * (mean.z - nearest_neighbour_z);
												cov[1][0]+=(mean.y - nearest_neighbour_y) * (mean.x - nearest_neighbour_x);
												cov[1][1]+=(mean.y - nearest_neighbour_y) * (mean.y - nearest_neighbour_y);
												cov[1][2]+=(mean.y - nearest_neighbour_y) * (mean.z - nearest_neighbour_z);
												cov[2][0]+=(mean.z - nearest_neighbour_z) * (mean.x - nearest_neighbour_x);
												cov[2][1]+=(mean.z - nearest_neighbour_z) * (mean.y - nearest_neighbour_y);
												cov[2][2]+=(mean.z - nearest_neighbour_z) * (mean.z - nearest_neighbour_z);
												number_of_nearest_neighbours++;
											}
										}
									}
								}
							}
						}
					}

					if(number_of_nearest_neighbours >= number_of_points_needed_for_plane_threshold)
					{
						cov[0][0]/=number_of_nearest_neighbours;
						cov[0][1]/=number_of_nearest_neighbours;
						cov[0][2]/=number_of_nearest_neighbours;
						cov[1][0]/=number_of_nearest_neighbours;
						cov[1][1]/=number_of_nearest_neighbours;
						cov[1][2]/=number_of_nearest_neighbours;
						cov[2][0]/=number_of_nearest_neighbours;
						cov[2][1]/=number_of_nearest_neighbours;
						cov[2][2]/=number_of_nearest_neighbours;

						double U[3][3], V[3][3];
						double SS[9];
						gpuSVD((double *)cov, (double *)U, (double *)SS, (double *)V);
						double _nx = (float)(U[0][1]*U[1][2] - U[0][2]*U[1][1]);
						double _ny = (float)(-(U[0][0]*U[1][2] - U[0][2]*U[1][0] ));
						double _nz = (float)(U[0][0]*U[1][1] - U[0][1]*U[1][0]);

						double lenght = sqrt(_nx*_nx + _ny*_ny + _nz*_nz);
						if(lenght==0)
						{
							d_point_cloud[hashed_index_of_point].normal_x = 0.0f;
							d_point_cloud[hashed_index_of_point].normal_y = 0.0f;
							d_point_cloud[hashed_index_of_point].normal_z = 0.0f;
						}else
						{
							d_point_cloud[hashed_index_of_point].normal_x = _nx/lenght;
							d_point_cloud[hashed_index_of_point].normal_y = _ny/lenght;
							d_point_cloud[hashed_index_of_point].normal_z = _nz/lenght;
							if( (SS[4]/SS[8]) > curvature_threshold)
							{
								d_point_cloud[hashed_index_of_point].label = LABEL_PLANE;
							}
						}
					}
					else
					{
						d_point_cloud[hashed_index_of_point].normal_x = 0.0f;
						d_point_cloud[hashed_index_of_point].normal_y = 0.0f;
						d_point_cloud[hashed_index_of_point].normal_z = 0.0f;
					}
				}
			}
		}
	}
}

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
	int number_of_points_needed_for_plane_threshold)
{
	cudaError_t err = cudaGetLastError();

	int blocks=number_of_points/threads+1;

	kernel_normalvectorcomputation_step1_fast<<<blocks,threads>>>(
		d_point_cloud,
		d_hashTable,
		d_mean,
		number_of_points,
		d_buckets,
		rgd_params,
		search_radius,
		max_number_considered_in_INNER_bucket,
		max_number_considered_in_OUTER_bucket);
	err = cudaDeviceSynchronize();
	if(err != ::cudaSuccess)return err;

	kernel_normalvectorcomputation_step2_fast_with_classification<<<blocks,threads>>>(
		d_point_cloud,
		d_hashTable,
		d_mean,
		number_of_points,
		d_buckets,
		rgd_params,
		search_radius,
		max_number_considered_in_INNER_bucket,
		max_number_considered_in_OUTER_bucket,
		curvature_threshold,
		number_of_points_needed_for_plane_threshold);
	err = cudaDeviceSynchronize();

	return err;
}

__global__ void kernel_semanticLabelingFloorCeiling(
			int threads,
			lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud,
			int number_of_points,
			float ground_Z_coordinate_threshold)
{
	int index_of_point = blockIdx.x * blockDim.x + threadIdx.x;
	if(index_of_point < number_of_points)
	{
		if(d_point_cloud[index_of_point].label == 0)
		{
			if(d_point_cloud[index_of_point].normal_z > 0.7 || d_point_cloud[index_of_point].normal_z < -0.7)
			{
				if(d_point_cloud[index_of_point].z < ground_Z_coordinate_threshold)
				{
					d_point_cloud[index_of_point].label = LABEL_GROUND;
				}else
				{
					d_point_cloud[index_of_point].label = LABEL_CEILING;
				}
			}
		}
	}
}

cudaError_t cudaSemanticLabelingFloorCeiling(
		int threads,
		lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud,
		int number_of_points,
		float ground_Z_coordinate_threshold)
{
	cudaError_t err = cudaGetLastError();
	int blocks=number_of_points/threads+1;

	kernel_semanticLabelingFloorCeiling<<<blocks,threads>>>(
			threads,
			d_point_cloud,
			number_of_points,
			ground_Z_coordinate_threshold);

	err = cudaDeviceSynchronize();
	return err;
}

__global__ void  kernel_change_nn_value_for_reduction(int *d_nearest_neighbour_indexes, unsigned int nop)
{
	int ind = blockIdx.x*blockDim.x + threadIdx.x;

	if (ind<nop)
	{
		if (d_nearest_neighbour_indexes[ind] < 0)
		{
			d_nearest_neighbour_indexes[ind] = 0;
		}
		else
		{
			d_nearest_neighbour_indexes[ind] = 1;
		}
	}
}

cudaError_t cudaCountNumberOfSemanticNearestNeighbours(
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
	int *d_nearest_neighbour_indexes,
	int &number_of_nn)
{
	cudaError_t err = cudaGetLastError();

	int blocks = number_of_points_second_point_cloud / threads + 1;

	kernel_semanticNearestNeighborSearch << <blocks, threads >> >(
		d_first_point_cloud,
		number_of_points_first_point_cloud,
		d_second_point_cloud,
		number_of_points_second_point_cloud,
		d_hashTable,
		d_buckets,
		rgd_params,
		search_radius,
		max_number_considered_in_INNER_bucket,
		max_number_considered_in_OUTER_bucket,
		d_nearest_neighbour_indexes);

	kernel_change_nn_value_for_reduction << <blocks, threads >> > (d_nearest_neighbour_indexes, number_of_points_second_point_cloud);

	thrust::device_ptr <int> dev_ptr_d_nearest_neighbour_indexes(d_nearest_neighbour_indexes);
	number_of_nn = thrust::reduce(dev_ptr_d_nearest_neighbour_indexes, dev_ptr_d_nearest_neighbour_indexes + number_of_points_second_point_cloud);


	err = cudaDeviceSynchronize();
	return err;
}

__global__ void kernel_cudaTransformPointCloud(lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud,
		int number_of_points,
		float r00, float r10, float r20, float r01, float r11, float r21, float r02, float r12, float r22, float t0, float t1, float t2)
{
	int ind = blockIdx.x*blockDim.x + threadIdx.x;

	if (ind<number_of_points)
	{
		lidar_pointcloud::PointXYZIRNLRGB d_p = d_point_cloud[ind];
		float vOut[3];
		vOut[0] = r00 * d_p.x + r01 * d_p.y + r02 * d_p.z + t0;
		vOut[1] = r10 * d_p.x + r11 * d_p.y + r12 * d_p.z + t1;
		vOut[2] = r20 * d_p.x + r21 * d_p.y + r22 * d_p.z + t2;

		d_p.x = vOut[0];
		d_p.y = vOut[1];
		d_p.z = vOut[2];

		vOut[0] = r00 * d_p.normal_x + r01 * d_p.normal_y + r02 * d_p.normal_z;
		vOut[1] = r10 * d_p.normal_x + r11 * d_p.normal_y + r12 * d_p.normal_z;
		vOut[2] = r20 * d_p.normal_x + r21 * d_p.normal_y + r22 * d_p.normal_z;

		d_p.normal_x = vOut[0];
		d_p.normal_y = vOut[1];
		d_p.normal_z = vOut[2];

		d_point_cloud[ind] = d_p;
	}
}

cudaError_t cudaTransformPointCloud(int threads, lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud,
		int number_of_points,
		float r00, float r10, float r20, float r01, float r11, float r21, float r02, float r12, float r22, float t0, float t1, float t2)
{
	kernel_cudaTransformPointCloud << <number_of_points / threads + 1, threads >> >
			(d_point_cloud, number_of_points, r00, r10, r20, r01, r11, r21, r02, r12, r22, t0, t1, t2);

	cudaDeviceSynchronize();
	return cudaGetLastError();
}

__global__ void kernel_cudaTransformPointCloud(lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud_in,
		int number_of_points_in,
		lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud_out,
		int number_of_points_out,
		float r00, float r10, float r20, float r01, float r11, float r21, float r02, float r12, float r22, float t0, float t1, float t2)
{
	int ind = blockIdx.x*blockDim.x + threadIdx.x;

	if ((ind < number_of_points_in) && (number_of_points_in == number_of_points_out))
	{
		lidar_pointcloud::PointXYZIRNLRGB d_p = d_point_cloud_in[ind];
		float vOut[3];
		vOut[0] = r00 * d_p.x + r01 * d_p.y + r02 * d_p.z + t0;
		vOut[1] = r10 * d_p.x + r11 * d_p.y + r12 * d_p.z + t1;
		vOut[2] = r20 * d_p.x + r21 * d_p.y + r22 * d_p.z + t2;

		d_p.x = vOut[0];
		d_p.y = vOut[1];
		d_p.z = vOut[2];

		vOut[0] = r00 * d_p.normal_x + r01 * d_p.normal_y + r02 * d_p.normal_z;
		vOut[1] = r10 * d_p.normal_x + r11 * d_p.normal_y + r12 * d_p.normal_z;
		vOut[2] = r20 * d_p.normal_x + r21 * d_p.normal_y + r22 * d_p.normal_z;

		d_p.normal_x = vOut[0];
		d_p.normal_y = vOut[1];
		d_p.normal_z = vOut[2];

		d_point_cloud_out[ind] = d_p;
	}
}

cudaError_t cudaTransformPointCloud(int threads, lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud_in,
		int number_of_points_in,
		lidar_pointcloud::PointXYZIRNLRGB *d_point_cloud_out,
		int number_of_points_out,
		float r00, float r10, float r20, float r01, float r11, float r21, float r02, float r12, float r22, float t0, float t1, float t2)
{
	kernel_cudaTransformPointCloud << <number_of_points_in / threads + 1, threads >> >
		(d_point_cloud_in, number_of_points_in, d_point_cloud_out, number_of_points_out, r00, r10, r20, r01, r11, r21, r02, r12, r22, t0, t1, t2);

	cudaDeviceSynchronize();
	return cudaGetLastError();
}
