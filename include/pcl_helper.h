#ifndef PCL_HELPER_H
#define PCL_HELPER_H

#include <pcl/io/stl_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h> // For future normal estimation
#include <vector>
#include <string>

// Function to load an STL file and convert it to a point cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromSTL(const std::string& filename);

// Function to slice the point cloud along the Z-axis
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slicePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_slices, double ransac_threshold);

// Function to approximate the parting line by connecting centroids of slices
pcl::PointCloud<pcl::PointXYZ>::Ptr approximatePartingLine(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& slices);

// Function to estimate normals - to be implemented later
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double search_radius);

// Function to detect undercuts - to be implemented later
std::vector<bool> detectUndercuts(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals);

#endif // PCL_HELPER_H