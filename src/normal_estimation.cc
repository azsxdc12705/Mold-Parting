#include "normal_estimation.h"

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double search_radius) {

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Use a KdTree for faster neighborhood searching
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Set the search radius (adjust as needed based on point cloud density)
    ne.setRadiusSearch(search_radius);

    // Compute the normals
    ne.compute(*cloud_normals);

    return cloud_normals;
}