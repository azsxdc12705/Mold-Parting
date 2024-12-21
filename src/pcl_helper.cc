#include "pcl_helper.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointCloudFromSTL(const std::string& filename) {
    pcl::PolygonMesh mesh;
    if (pcl::io::loadSTLFile(filename, mesh) == -1) {
        PCL_ERROR("Error loading STL file: %s\n", filename.c_str());
        return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    return cloud;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slicePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_slices, double ransac_threshold) {

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slices;
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    float slice_spacing = (maxPt.z - minPt.z) / num_slices;

    for (int i = 0; i < num_slices; ++i) {
        float z = minPt.z + i * slice_spacing;

        // Define the plane coefficients: ax + by + cz + d = 0
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0] = 0; // a (normal.x)
        coefficients->values[1] = 0; // b (normal.y)
        coefficients->values[2] = 1.0; // c (normal.z)
        coefficients->values[3] = -z; // d

        // Use SACSegmentation to fit a plane model and extract inliers
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_threshold);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Extract the inliers (points close to the plane)
        pcl::PointCloud<pcl::PointXYZ>::Ptr slice_i(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*slice_i);

        if (!slice_i->empty()) {
            slices.push_back(slice_i);
        }
    }

    return slices;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr approximatePartingLine(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& slices) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr parting_line(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& slice : slices) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*slice, centroid);
        parting_line->push_back(pcl::PointXYZ(centroid[0], centroid[1], centroid[2]));
    }
    return parting_line;
}

// Stub for normal estimation - to be implemented later
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double search_radius) {
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(search_radius);

    ne.compute(*cloud_normals);

    return cloud_normals;
}

// Stub for undercut detection - to be implemented later
std::vector<bool> detectUndercuts(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals) {

    std::vector<bool> undercut_flags;
    // Placeholder: Mark every other segment as a potential undercut for now
    for (size_t i = 0; i < parting_line->size() - 1; ++i) {
        undercut_flags.push_back(i % 2 == 0);
    }
    return undercut_flags;
}