cmake_minimum_required(VERSION 3.14)
project(dufomap_tools
	VERSION 1.0.0
	DESCRIPTION "DUFOMap tools for data processing"
	LANGUAGES CXX
)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Glog REQUIRED)
find_package(gflags REQUIRED)
set(LOG_LIB glog::glog gflags)
find_package(PCL REQUIRED)

# import our timer library
include_directories(../lidar_localizer/include)

function(create_executable target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} ${PCL_LIBRARIES} ${LOG_LIB})
endfunction()


# Find the required packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs tf2 tf2_ros tf2_eigen)
find_package(rosbag QUIET)

# Add the executable and link the libraries for read_bag_example only if rosbag is found
if(rosbag_FOUND)
	# Include the directories
	include_directories(${catkin_INCLUDE_DIRS})
	add_executable(bag2pcd src/extract_pcdFbag_topic.cpp)
	target_link_libraries(bag2pcd ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${PCL_LIBRARIES} ${LOG_LIB})
	add_executable(bag2pcd_tf src/extract_pcdFbag_tf.cpp)
	target_link_libraries(bag2pcd_tf ${catkin_LIBRARIES} ${rosbag_LIBRARIES} ${PCL_LIBRARIES} ${LOG_LIB})
else()
	message(WARNING "rosbag package not found. Skipping read_bag_example.")
endif()
