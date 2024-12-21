#ifndef NORMAL_ESTIMATION_H
#define NORMAL_ESTIMATION_H

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h> // Include for KdTree

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double search_radius);

#endif // NORMAL_ESTIMATION_H