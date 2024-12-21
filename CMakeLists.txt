cmake_minimum_required(VERSION 3.5)  # Require CMake version 3.5 or higher
project(parting_line)

# Find the Point Cloud Library (PCL)
find_package(PCL 1.7 REQUIRED)  # Require PCL 1.7 or newer

# Include the directories for PCL and your project
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

# Link the directories for PCL libraries
link_directories(${PCL_LIBRARY_DIRS})

# Add any necessary compiler definitions from PCL
add_definitions(${PCL_DEFINITIONS})

# Add the source files for the executable
add_executable(parting_line
    src/parting_line.cc
    src/pcl_helper.cc
    src/pcl_visualizer.cc
    src/utils.cc
    src/normal_estimation.cc
    src/undercut_detection.cc
)

# Link the executable to the required PCL libraries
target_link_libraries(parting_line
    ${PCL_LIBRARIES}
)