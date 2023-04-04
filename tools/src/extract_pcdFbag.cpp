/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 18:29
 * Description: extract pcd file and also insert the pose in PCD VIEWPOINT Field
 * So that we don't need pose.csv file etc.
 * 
 * Input: ROS bag file, LiDAR topic name, Odom topic name [Default is for simple_ndt_slam]
 *        which is /odom_lidar, /auto_odom
 * Output: PCD files!
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <glog/logging.h>
#include "timer.h"


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::INFO);

  // if no argv, return 0
  if (argc < 3) {
    LOG(ERROR) << "Please input the rosbag path and the folder to save pcd files";
    return 1;
  }
  std::string rosbag_path = argv[1];
  std::string save_pcd_folder = argv[2];
  int save_map_pcd = 0;
  if (argc > 4){
    save_map_pcd = argv[3];
    if(save_map_pcd == 1){
      LOG(INFO) << "We will save map pcd file";
    }
  }
  // check if the file exists
  if (!std::filesystem::exists(rosbag_path)) {
    LOG(ERROR) << "File does not exist: " << rosbag_path;
    return 1;
  }
  // create the folder if not exists
  if (!std::filesystem::exists(save_pcd_folder)) {
    std::filesystem::create_directory(save_pcd_folder);
    LOG(INFO) << "Create folder: " << save_pcd_folder;
  }

  LOG(INFO) << "we will read bag through: " << rosbag_path;
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  
  std::string odom_topic = "/auto_odom";
  std::string pc2_topic = "/odom_lidar";
  std::vector<std::string> topics = {odom_topic, pc2_topic};
  LOG(INFO) << "We will read odom topic: " << ANSI_BOLD<< odom_topic  << ANSI_RESET 
            << " and pc2 topic: " << ANSI_BOLD << pc2_topic << ANSI_RESET;

  TIC;
  // Create a view for the bag with the desired topic
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_map(new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<float> pose(7, 0.0);
  bool pc_rec = false, odom_rec = false;
  int count = 0;
  for(const rosbag::MessageInstance& m : view)
  {
    if(pc_rec && odom_rec)
    {
      // transform the pcd
      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      transform.block<3, 3>(0, 0) = Eigen::Quaternionf(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix();
      transform.block<3, 1>(0, 3) = Eigen::Vector3f(pose[0], pose[1], pose[2]);
      pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, transform);
      
      if(save_map_pcd == 1)
        pcl_cloud_map->insert(pcl_cloud_map->end(), pcl_cloud->begin(), pcl_cloud->end());

      // set the viewpoint -> CHECK PCD VIEWPOINT FIELD
      pcl_cloud->sensor_origin_ = Eigen::Vector4f(pose[0], pose[1], pose[2], 0.0);
      pcl_cloud->sensor_orientation_ = Eigen::Quaternionf(pose[6], pose[3], pose[4], pose[5]); // w, x, y, z

      // save the pcd
      std::ostringstream tmp_filename;
      tmp_filename << save_pcd_folder << "/" << std::setfill('0') << std::setw(6) << count << ".pcd";
      std::string pcd_file = tmp_filename.str();
      pcl::io::savePCDFileBinary(pcd_file, *pcl_cloud);

      pc_rec = false;
      odom_rec = false;
      count++;
    }
    if(m.getTopic() == pc2_topic)
    {
      sensor_msgs::PointCloud2::ConstPtr pc2 = m.instantiate<sensor_msgs::PointCloud2>();
      if(pc2 != nullptr)
      {
        pcl::fromROSMsg(*pc2, *pcl_cloud);
        pc_rec = true;
      }
    }
    else if(m.getTopic() == odom_topic)
    {
      nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
      if(odom != nullptr)
      {
        odom_rec = true;
        pose[0] = odom->pose.pose.position.x;
        pose[1] = odom->pose.pose.position.y;
        pose[2] = odom->pose.pose.position.z;
        pose[3] = odom->pose.pose.orientation.x;
        pose[4] = odom->pose.pose.orientation.y;
        pose[5] = odom->pose.pose.orientation.z;
        pose[6] = odom->pose.pose.orientation.w;
      }
    }
  }
  TOC("Extract pcd files from rosbag", true);
  if(save_map_pcd == 1)
  {
    std::ostringstream tmp_filename;
    // last folder in the path
    std::string folder_name = save_pcd_folder.substr(save_pcd_folder.find_last_of("/\\") + 1);
    tmp_filename << folder_name << "/" << "raw_map.pcd";
    std::string pcd_file = tmp_filename.str();
    pcl::io::savePCDFileBinary(pcd_file, *pcl_cloud_map);
    LOG(INFO) << "Save raw map pcd file: " << pcd_file << " with " << pcl_cloud_map->size() << " points";
  }

  LOG(INFO) << "Total pcd files: " << count << ANSI_GREEN << " Runned successfully!" << ANSI_RESET;
  bag.close();

  return 0;
}
