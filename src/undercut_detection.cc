#include "undercut_detection.h"
#include <pcl/common/angles.h>

std::vector<bool> detectUndercuts(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double tolerance) {

    std::vector<bool> undercut_flags(parting_line->size() - 1, false);

    if (parting_line->empty() || normals->empty()) {
        return undercut_flags;
    }

    // Assume Z-axis as the parting direction
    Eigen::Vector3f parting_direction(0, 0, 1);

    for (size_t i = 0; i < parting_line->size() - 1; ++i) {
        // Find the closest point in the cloud for the current centroid
        pcl::PointXYZ centroid = parting_line->points[i];
        int closest_index = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t j = 0; j < cloud->size(); ++j)
        {
            double dist = pcl::euclideanDistance(centroid, cloud->points[j]);
            if (dist < min_dist)
            {
                min_dist = dist;
                closest_index = j;
            }
        }
        
        if (closest_index == -1) continue;

        // Get the normal at the closest point
        pcl::Normal normal = normals->points[closest_index];
        Eigen::Vector3f normal_vec(normal.normal_x, normal.normal_y, normal.normal_z);

        // Check for undercut
        if (normal_vec.dot(parting_direction) < tolerance) {
            undercut_flags[i] = true;
        }
    }

    return undercut_flags;
}