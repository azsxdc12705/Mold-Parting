#ifndef PCL_VISUALIZER_H
#define PCL_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>

class PCLVisualizer {
public:
    PCLVisualizer();

    void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& id = "cloud");

    void visualizePartingLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line);

    void visualizeSlicingPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_slices, double ransac_threshold);

    void visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr& normals); //

    void visualizeUndercuts(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<bool>& undercut_flags); //

    pcl::visualization::PCLVisualizer::Ptr getViewer();

private:
    pcl::visualization::PCLVisualizer::Ptr viewer;
};

#endif // PCL_VISUALIZER_H