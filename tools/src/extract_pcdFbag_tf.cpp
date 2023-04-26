/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-26 14:33
 * Description: extract pcd file and also insert the pose in PCD VIEWPOINT Field
 * So that we don't need pose.csv file etc.
 *
 * Input: ROS bag file, LiDAR topic name, Output: PCD files!
 */

#include <glog/logging.h>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "timer.h"

int main(int argc, char** argv) {
  // Initialize the ROS system
  ros::init(argc, argv, "bag2pcd_tf");

  // Create a NodeHandle object
  ros::NodeHandle nh;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::INFO);

  // if no argv, return 0
  if(argc < 4){
    LOG(ERROR) << "./bag2pcd_tf <rosbag_path> <save_pcd_folder> <pc2_topic_name>";
    return 1;
  }
  std::string rosbag_path = argv[1];
  std::string save_pcd_folder = argv[2];
  std::string pc2_topic = argv[3];

  int save_map_pcd = 0;
  if (argc > 4) {
    save_map_pcd = std::stoi(argv[4]);
    if (save_map_pcd == 1) {
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
  if(!std::filesystem::exists(save_pcd_folder+"/pcd"))
    std::filesystem::create_directory(save_pcd_folder+"/pcd");
  LOG(INFO) << "we will read bag through: " << rosbag_path;
  rosbag::Bag bag;
  bag.open(rosbag_path, rosbag::bagmode::Read);

  std::vector<std::string> topics = {pc2_topic, "/tf", "/tf_static"};
  LOG(INFO) << "We will read pc2 topic: " << ANSI_BOLD << pc2_topic
            << ANSI_RESET << " and pose info from tf topic directly";

  TIC;
  // Create a view for the bag with the desired topic
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_map(
      new pcl::PointCloud<pcl::PointXYZI>());
  std::vector<float> pose(7, 0.0);
  int count = 0;

  ros::Duration cache_length(3600.0);// 3600s = 1h HARD CODE!
  tf2_ros::Buffer tf_buffer(cache_length);
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::TransformStamped static_transform;
  bool static_frame_set = false;
  for (const rosbag::MessageInstance& m : view) {
    if (m.isType<tf2_msgs::TFMessage>()) {
      tf2_msgs::TFMessage::ConstPtr tf_msg =
          m.instantiate<tf2_msgs::TFMessage>();
      if (tf_msg != nullptr) {

        for (const auto& transform : tf_msg->transforms) {
          try {
            // FIXME: set velodyne frame and livox frame HARD CODE! HERE
            if(transform.child_frame_id == "livox_frame"){
              static_transform = transform;
              static_frame_set = true;
              continue;
            }

            if(transform.header.frame_id == "map" && static_frame_set){
              static_transform.header.stamp = transform.header.stamp;
              tf_buffer.setTransform(static_transform, "static_transform");
            }
            tf_buffer.setTransform(transform, "bag_tf");
          } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
          }
        }
      }
    }
  }

  bool pc_rec = false;
  for(const rosbag::MessageInstance& m : view)
  {
    if(pc_rec)
    {
      // transform the pcd
      Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
      transform.block<3, 3>(0, 0) = Eigen::Quaternionf(pose[6], pose[3],
      pose[4], pose[5]).toRotationMatrix(); transform.block<3, 1>(0, 3) =
      Eigen::Vector3f(pose[0], pose[1], pose[2]);
      pcl::transformPointCloud(*pcl_cloud, *pcl_cloud, transform);

      if(save_map_pcd == 1)
        pcl_cloud_map->insert(pcl_cloud_map->end(), pcl_cloud->begin(),
        pcl_cloud->end());

      // set the viewpoint -> CHECK PCD VIEWPOINT FIELD
      pcl_cloud->sensor_origin_ = Eigen::Vector4f(pose[0], pose[1], pose[2],
      0.0); pcl_cloud->sensor_orientation_ = Eigen::Quaternionf(pose[6],
      pose[3], pose[4], pose[5]); // w, x, y, z

      // save the pcd
      std::ostringstream tmp_filename;
      tmp_filename << save_pcd_folder << "/pcd/" << std::setfill('0') <<
      std::setw(6) << count << ".pcd"; std::string pcd_file =
      tmp_filename.str(); pcl::io::savePCDFileBinary(pcd_file, *pcl_cloud);

      pc_rec = false;
      count++;
    }
    if(m.getTopic() == pc2_topic)
    {
      sensor_msgs::PointCloud2::ConstPtr pc2 =
      m.instantiate<sensor_msgs::PointCloud2>();

      if(pc2 != nullptr)
      {
        pcl::fromROSMsg(*pc2, *pcl_cloud);
        try{
          geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform("map", pc2->header.frame_id, pc2->header.stamp);
          // Position
          pose[0] = transform.transform.translation.x;
          pose[1] = transform.transform.translation.y;
          pose[2] = transform.transform.translation.z;

          // Orientation
          pose[3] = transform.transform.rotation.x;
          pose[4] = transform.transform.rotation.y;
          pose[5] = transform.transform.rotation.z;
          pose[6] = transform.transform.rotation.w;
          pc_rec = true;
        }
        catch (tf2::TransformException& ex) {
          ROS_WARN("%s, We will skip this frame", ex.what());
          continue;
        }
      }
    }
  }
  TOC("Extract pcd files from rosbag", true);
  if(save_map_pcd == 1)
  {
    std::ostringstream tmp_filename;
    // last folder in the path
    std::string folder_name =
    save_pcd_folder.substr(save_pcd_folder.find_last_of("/\\") + 1);
    tmp_filename << folder_name << "/" << "raw_map.pcd";
    std::string pcd_file = tmp_filename.str();
    LOG(INFO) << pcd_file;
    pcl::io::savePCDFileBinary(pcd_file, *pcl_cloud_map);
    LOG(INFO) << "Save raw map pcd file: " << pcd_file << " with " <<
    pcl_cloud_map->size() << " points";
  }

  LOG(INFO) << "Total pcd files: " << count << ANSI_GREEN << " Run successfully!" << ANSI_RESET; bag.close();

  return 0;
}
