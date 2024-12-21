#ifndef UNDERCUT_DETECTION_H
#define UNDERCUT_DETECTION_H

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>

std::vector<bool> detectUndercuts(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double tolerance = 0.8); // Add tolerance as a parameter

#endif // UNDERCUT_DETECTION_H