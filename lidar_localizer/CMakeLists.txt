cmake_minimum_required(VERSION 2.8.3)
project(lidar_localizer)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
# NOTE: CHOOSE THE O2/O3 is really important for speed! check more on optimization on compile
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/ndt_mapper.cpp
        )

###############
# Executables #
###############

cs_add_executable(ndt_mapping
        ndt_mapping_node.cpp
        )

cs_add_executable(queue_counter
        queue_counter_node.cpp
        )

target_link_libraries(ndt_mapping ${PROJECT_NAME} 
        glog::glog
        )

target_link_libraries(queue_counter ${PROJECT_NAME} 
        glog::glog
        )
if(NOT (PCL_VERSION VERSION_LESS "1.7.2"))
  set_target_properties(ndt_mapping PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif()

###########
# Export #
###########

cs_install()
cs_export()

