cmake_minimum_required(VERSION 2.8.3)
project(loop_closing_detector)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    tf
    cv_bridge
    image_geometry
	std_msgs
	sensor_msgs
	nav_msgs
	geometry_msgs
	image_transport
	message_filters
	std_srvs
	cmake_modules)

# Dependencies - Boost:
find_package(Boost REQUIRED COMPONENTS signals filesystem system)
include_directories(${Boost_INCLUDE_DIRS})

# Dependencies - Eigen:
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

# Dependencies - Libhaloc:
find_package(libhaloc REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package()

# Include directories
include_directories(${catkin_INCLUDE_DIRS}
					${Eigen_INCLUDE_DIRS}
					${libhaloc_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS}
					${SUITESPARSE_INCLUDE_DIRS}
					${G2O_INCLUDE_DIRS}
					${PCL_INCLUDE_DIRS})

# Add the examples
add_executable(loop_closing_detector
			   src/loop_closing_detector.cpp)
target_link_libraries(loop_closing_detector
	                  ${eigen_LIBRARIES}
					  ${libhaloc_LIBRARIES}
					  ${OpenCV_LIBRARIES}
					  ${SUITESPARSE_LIBRARIES}
					  ${G2O_LIBRARIES}
					  ${catkin_LIBRARIES})