#ifndef __SEMANTIC_POINT_TYPES_H__
#define __SEMANTIC_POINT_TYPES_H__

#include <pcl/point_types.h>

namespace lidar_pointcloud
{
	struct PointXYZIRNLRGB
	{
		float x;
		float y;
		float z;
		float intensity;
		uint16_t   ring;
		float normal_x;
		float normal_y;
		float normal_z;
		int   label;
		float rgb;
	};

};

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_pointcloud::PointXYZIRNLRGB,
								(float, x, x)
								(float, y, y)
								(float, z, z)
								(float, intensity, intensity)
								(uint16_t, ring, ring)
								(float, normal_x, normal_x)
								(float, normal_y, normal_y)
								(float, normal_z, normal_z)
								(int, label, label)
								(float, rgb, rgb))



#endif


