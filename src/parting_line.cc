#include "pcl_helper.h"
#include "pcl_visualizer.h"
#include "normal_estimation.h"
#include "undercut_detection.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: ./parting_line <stl_file_path> <num_slices> <ransac_threshold>\n";
    return 1;
  }

  std::string stl_file = argv[1];
  int num_slices = (argc > 2) ? std::stoi(argv[2]) : 10;
  double ransac_threshold = (argc > 3) ? std::stod(argv[3]) : 0.001;

  // 1. Load STL and convert to point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPointCloudFromSTL(stl_file);
  if (!cloud) {
    std::cerr << "Error loading STL file.\n";
    return 1;
  }

  // 2. Estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(cloud, 0.03);

  // 3. Slice the point cloud
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> slices =
      slicePointCloud(cloud, num_slices, ransac_threshold);

  // 4. Approximate the parting line
  pcl::PointCloud<pcl::PointXYZ>::Ptr parting_line = approximatePartingLine(slices);

  // 5. Perform simplified undercut detection - Uncomment when you implement it
  std::vector<bool> undercut_flags = detectUndercuts(parting_line, normals);

  // 6. Visualization
  PCLVisualizer viewer;
  viewer.visualizePointCloud(cloud,"cloud");
  viewer.visualizeSlicingPlane(cloud, num_slices, ransac_threshold);
  viewer.visualizePartingLine(parting_line); // Pass normals and undercut_flags if calculated
  viewer.visualizeNormals(cloud, normals);
  viewer.visualizeUndercuts(parting_line, undercut_flags);
  
  // Keep the visualizer open until it's closed
  while (!viewer.getViewer()->wasStopped()) {
      viewer.getViewer()->spinOnce();
  }

  return 0;
}