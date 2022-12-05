/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: node file, please go to the ndt_mapper.cpp for more detail
 */

#include "ndt_mapper.h"
#include <glog/logging.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  // Start Ros.
  ros::init(argc, argv, "ndt_mapping", ros::init_options::NoSigintHandler);

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;

  // Setup node.
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  ndt_mapping::NDTMapper mapper(nh, nh_private);

  // // Setup spinning.
  ros::AsyncSpinner spinner(mapper.getConfig().ros_spinner_threads); //
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
