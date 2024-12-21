#include "pcl_visualizer.h"

PCLVisualizer::PCLVisualizer() : viewer(new pcl::visualization::PCLVisualizer("Parting Line Visualizer")) {
    viewer->setBackgroundColor(0, 0, 0); // Set background to black
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 0, 0, -1, 0);
}

void PCLVisualizer::visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& id) {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id); // Set point size
}

void PCLVisualizer::visualizePartingLine(const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line) {
    // Visualize the parting line as a sequence of connected line segments (red)
    for (size_t i = 0; i < parting_line->size() - 1; ++i) {
        std::string line_id = "line" + std::to_string(i);
        viewer->addLine<pcl::PointXYZ>(parting_line->points[i], parting_line->points[i + 1], 255, 0, 0, line_id);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_id); // Set line width
    }

    // Add small spheres at centroid points for better visualization
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(parting_line, 255, 0, 0);
    viewer->addPointCloud(parting_line, red, "centroids");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "centroids");
}

void PCLVisualizer::visualizeSlicingPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num_slices, double ransac_threshold){
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
        std::string plane_id = "plane" + std::to_string(i);
        viewer->addPlane(*coefficients, plane_id);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.2, plane_id); // Adjust opacity as needed
    }

}


void PCLVisualizer::visualizeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                    const pcl::PointCloud<pcl::Normal>::Ptr& normals) {
    // Visualize normals (e.g., using arrows)
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
}

void PCLVisualizer::visualizeUndercuts(const pcl::PointCloud<pcl::PointXYZ>::Ptr& parting_line,
                                         const std::vector<bool>& undercut_flags) {
     for (size_t i = 0; i < undercut_flags.size(); ++i) {
        if (undercut_flags[i]) {
            std::string sphere_id = "undercut" + std::to_string(i);
            // Assuming your parting line points are close to the undercut region
            viewer->addSphere(parting_line->points[i], 0.02, 255, 0, 0, sphere_id); // Red sphere for undercuts
        }
    }
}
pcl::visualization::PCLVisualizer::Ptr PCLVisualizer::getViewer() {
    return viewer;
}