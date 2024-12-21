# Point Cloud Parting Line Approximation

This project implements a simplified approach to parting line approximation from point clouds, inspired by the slicing algorithm in [Wong et al., 1998]. It leverages the Point Cloud Library (PCL) for point cloud processing and emphasizes core computer vision concepts.

## Overview

The program takes an STL file as input, converts it to a point cloud, slices it along the Z-axis, calculates the centroid of each slice, and connects these centroids to approximate the parting line. It also includes basic undercut detection based on surface normal estimation.

## Dependencies

*   **PCL (Point Cloud Library):** Version 1.7 or higher is recommended. PCL is used for point cloud I/O, processing, normal estimation, and visualization. Installation instructions can be found on the PCL website: [http://pointclouds.org/](http://pointclouds.org/)
*   **CMake:** Required for building the project.
*   **A C++ Compiler:** A C++11 compliant compiler is needed (e.g., g++, clang++).
*   **Eigen3:** Used for linear algebra operations. Install it with: `sudo apt-get install libeigen3-dev`

## Building

1. **Clone the repository:**

    ```bash
    git clone https://github.com/your-username/point-cloud-parting-line.git
    ```

    (Replace with your actual repository link if you have one)

2. **Create a build directory:**

    ```bash
    cd point-cloud-parting-line
    mkdir build
    cd build
    ```

3. **Configure the project with CMake:**

    ```bash
    cmake ..
    ```

4. **Build the project:**

    ```bash
    make
    ```

## Running

1. **Place your STL file** (e.g., `your_model.stl`) in the `data/` directory.
2. **Run the executable:**

    ```bash
    ./parting_line data/your_model.stl <num_slices> <ransac_threshold> <search_radius>
    ```

    *   **`<num_slices>`:** The number of slicing planes (default: 10).
    *   **`<ransac_threshold>`:** The distance threshold for RANSAC plane fitting during slicing (default: 0.001).
    *   **`<search_radius>`:** The radius to search the point for normal calculation.

    **Example:**

    ```bash
    ./parting_line data/your_model.stl 15 0.005 0.03
    ```

    This will open a PCL visualizer window displaying the point cloud, approximated parting line, estimated normals, and potential undercuts.

## Modules

*   **`parting_line.cpp`:** Main program. Handles command-line arguments, calls other modules, and manages the visualization loop.
*   **`pcl_helper.h/cpp`:** Contains helper functions for loading and preprocessing point clouds, slicing, centroid calculation, and parting line creation.
*   **`pcl_visualizer.h/cpp`:** Contains functions for visualizing the point cloud, parting line, normals, and undercuts.
*   **`normal_estimation.h/cpp`:** Contains functions for estimating surface normals using PCL.
*   **`undercut_detection.h/cpp`:** Contains functions for performing simplified undercut detection.
*   **`utils.h/cpp`:** General utility functions (if needed).

## Key Parameters

*   **`num_slices`:** The number of slicing planes. Increasing this generally improves the accuracy of the parting line approximation but also increases computation time.
*   **`ransac_threshold`:** The distance threshold used in RANSAC segmentation for slicing. Adjust this parameter based on the scale of your point cloud and the desired accuracy. A smaller threshold means a tighter fit to the slicing plane.
*   **`search_radius`:**  The radius used for searching nearest neighbors during normal estimation. Adjust this based on the density of your point cloud.
*   **`tolerance`:** The tolerance used in the dot product comparison for undercut detection (in `detectUndercuts()`).

## Output

The program visualizes the following in a PCLVisualizer window:

*   The original point cloud.
*   The slicing planes.
*   The approximated parting line (connecting the centroids of the slices).
*   Estimated surface normals.
*   Highlighted potential undercut regions (spheres for visualization).

## Current Limitations

*   **Fixed Parting Direction:** The parting direction is fixed along the Z-axis.
*   **Simplified Parting Line:** The parting line is approximated by connecting centroids, which might not be accurate for complex shapes.
*   **Basic Undercut Detection:** Undercut detection is simplified and relies on normal estimation accuracy.

## Future Work

*   **Optimize Parting Direction:** Explore different parting directions to find a more optimal parting line.
*   **Improve Parting Line Approximation:** Implement more sophisticated contour extraction from 2D slices for a more accurate parting line.
*   **Advanced Undercut Detection and Handling:** Incorporate more robust undercut detection and handling algorithms.
*   **Multi-Piece Mold Design:** Extend the system to handle cases requiring multi-piece molds.
*   **Integrate with Topology Optimization:** Combine this approach with topology optimization techniques for overall mold optimization.

## References

*   Wong, T., Tan, S. T., and Sze, W. S. (1998). Parting Line Formation by Slicing a 3D CAD Model. Engineering with Computers, 14(4), 330–343.
*   Rusu, R. B., \& Cousins, S. (2011). 3D is here: Point Cloud Library (PCL). In IEEE International Conference on Robotics and Automation (ICRA) (pp. 1-4).

## Acknowledgements

This project was inspired by the work of Wong et al. (1998) and utilizes the Point Cloud Library (PCL).
