#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

typedef std::pair<pcl::PointXYZI, pcl::PointXYZI> PointLine;
