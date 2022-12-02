/**
 * Copyright (C) 2022, IADC, Hong Kong University of Science and Technology
 * MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2022-11-05
 * Description: the main file for ndt cal the translation between to points and
 * then publish the msg
 */

#include <omp.h>

// our define
#include "ndt_mapper.h"
#include "timer.h"

namespace ndt_mapping {

NDTMapper::NDTMapper(const ros::NodeHandle &nh,
                     const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  // Setup all components of the panoptic mapper.
  setConfig();
  setupMembers();
  setupRos();
}

void NDTMapper::setupRos() {
  getTF(); // receive the tf from base_link to LiDAR, default is 0,0,0,0,0,0

  ndt_map_pub = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_map", 10);
  path_viz_pub = nh_.advertise<nav_msgs::Path>("/robot_path", 10);
  current_pose_pub =
      nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 10);
  current_lidar_pub =
      nh_.advertise<sensor_msgs::PointCloud2>(_odom_lidar_topic, 10);
  current_odom_pub = nh_.advertise<nav_msgs::Odometry>(_output_odom_topic, 10);
  offset_odom_pub = nh_.advertise<nav_msgs::Odometry>(_output_offset_odom, 10);

  points_sub =
      nh_.subscribe(_lidar_topic, 10, &NDTMapper::points_callback, this);
  odom_sub = nh_.subscribe(_odom_topic, 10, &NDTMapper::odom_callback, this);
  imu_sub = nh_.subscribe(_imu_topic, 10, &NDTMapper::imu_callback, this);

  save_map_srv = nh_.advertiseService("/save_map", &NDTMapper::saveMap, this);
}

void NDTMapper::points_callback(
    const sensor_msgs::PointCloud2::ConstPtr &input) {
  common::Timer timer_total;
  current_scan_time = input->header.stamp;

  TIC;
  // 0. filter by min and max range
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr = filterPoints(input);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  // 1. Apply voxelgrid filter
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(config_.voxel_leaf_size,
                                config_.voxel_leaf_size,
                                config_.voxel_leaf_size);
  voxel_grid_filter.setInputCloud(scan_ptr);
  voxel_grid_filter.filter(*filtered_scan_ptr);
  TOC("FILTER POINT BY RANGE AND VOXEL", _debug_print);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity()),
      t_base_link(Eigen::Matrix4f::Identity());

  // 2. Using pcl NDT to calculate translation matrix
  t_localizer =
      calTranslation(scan_ptr, transformed_scan_ptr, filtered_scan_ptr);

  // 3. Pub all result message
  tf::Matrix3x3 mat_l, mat_b;
  t_base_link = t_localizer * tf_ltob; // from lidar to base_link fix matrix
  mat_l = setValue(t_localizer);
  mat_b = setValue(t_base_link);

  guess_lidar_pose.setPose(t_localizer, mat_l);
  guess_base_pose.setPose(t_base_link, mat_b);
  current_pose = guess_base_pose;

  // pub tf message
  tf::Transform transform_;
  tf::Quaternion q_base, q_;
  transform_.setOrigin(
      tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
  q_base.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
  transform_.setRotation(q_base);
  br_.sendTransform(
      tf::StampedTransform(transform_, current_scan_time, "map", "base_link"));
  if (no_b2l_tf) {
    transform_.setOrigin(tf::Vector3(b2l_tf[0], b2l_tf[1], b2l_tf[2]));
    q_.setRPY(b2l_tf[3], b2l_tf[4], b2l_tf[5]);
    transform_.setRotation(q_);
    br_.sendTransform(tf::StampedTransform(transform_, current_scan_time,
                                           "base_link", lidar_frame));
    br_.sendTransform(tf::StampedTransform(transform_, current_scan_time,
                                           "base_link", "odom"));
  }

  double shift = (current_pose - added_pose).calDistance();
  if (shift >= config_.min_add_scan_shift) {
    TRE;
    added_pose = current_pose;

    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
    map += *transformed_scan_ptr;

    if (_incremental_voxel_update == true && config_.save_frame_point == -1)
      ndt_.updateVoxelGrid(transformed_scan_ptr); // update voxel directly
    else
      ndt_.setInputTarget(map.makeShared()); // set for next

    // ndt_.setInputTarget(map_ptr); // for pcl default lib
    TOC("NDT UPDATE INPUT", _debug_print);

    // output the single ndt map, rviz set delay
    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*transformed_scan_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    map_msg_ptr->header.stamp = current_scan_time;
    ndt_map_pub.publish(*map_msg_ptr);

    // filter for fast localization
    if (config_.save_frame_point != -1) {
      int one_scan_points_num = (*scan_ptr).size();
      int saved_points_num = one_scan_points_num * config_.save_frame_point;

      // remove the mapping data to release the memory
      if (map.size() > saved_points_num) {
        LOG(INFO) << "Number of earse points: "
                  << (map.size() - saved_points_num) << " points";
        auto oldest = map.begin();
        auto end_num = map.begin() + (map.size() - saved_points_num);
        map.erase(oldest, end_num);
      }
    }
  }

  pubOtherMsgs(input); // publish msg including odom, lidar, path_vis
  timer_total.End("WHOLE PROCESS", _debug_print);
  if (_debug_print) {
    std::cout
        << "-----------------------------------------------------------------"
        << std::endl;
    std::cout << "Sequence number: " << input->header.seq << std::endl;
    std::cout << "Number of scan points: " << scan_ptr->size() << " points."
              << std::endl;
    std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size()
              << " points." << std::endl;
    if (transformed_scan_ptr->points.size() != 0)
      std::cout << "transformed_scan_ptr: "
                << transformed_scan_ptr->points.size() << " points."
                << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "NDT has converged: " << has_converged << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "Number of iteration: " << final_num_iteration << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", "
              << current_pose.z << ", " << current_pose.roll << ", "
              << current_pose.pitch << ", " << current_pose.yaw << ")"
              << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "shift: " << shift << std::endl;
    std::cout
        << "-----------------------------------------------------------------"
        << std::endl;
  }
}

void NDTMapper::odom_callback(const nav_msgs::Odometry::ConstPtr &input) {
  static Pose initial_odom_ = MsgPose2Pose(input);
  Pose offset = MsgPose2Pose(input) - initial_odom_;

  nav_msgs::Odometry odom_to_pub;
  odom_to_pub.header.stamp = current_scan_time;
  odom_to_pub.header.frame_id = "map";
  odom_to_pub.child_frame_id = lidar_frame;
  odom_to_pub.twist = input->twist;

  // please check whether the offset correct
  if (_odom_inverse) {
    offset.x = -offset.x;
    offset.y = -offset.y;
    odom_to_pub.twist.twist.linear.x = -input->twist.twist.linear.x;
  }

  Pose2MsgPose(&(odom_to_pub.pose.pose), offset);
  offset_odom_pub.publish(odom_to_pub);
  // TODO
}

void NDTMapper::imu_callback(const sensor_msgs::Imu::Ptr &input) {
  // TODO
}

Eigen::Matrix4f NDTMapper::Pose2Matrix(const Pose &p) {
  Eigen::AngleAxisf rotation_x(p.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rotation_y(p.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rotation_z(p.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f translation(p.x, p.y, p.z);

  Eigen::Matrix4f poseMatrix =
      (translation * rotation_z * rotation_y * rotation_x).matrix();
  return poseMatrix;
}

Pose NDTMapper::MsgPose2Pose(const nav_msgs::Odometry::ConstPtr &input) {
  Pose res;

  Eigen::Quaterniond q(
      input->pose.pose.orientation.w, input->pose.pose.orientation.x,
      input->pose.pose.orientation.y, input->pose.pose.orientation.z);
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

  res.x = input->pose.pose.position.x;
  res.y = input->pose.pose.position.y;
  res.z = input->pose.pose.position.z;
  res.roll = euler[0];
  res.pitch = euler[1];
  res.yaw = euler[2];

  return res;
}

template <typename Type>
void NDTMapper::Pose2MsgPose(Type msg_ptr, const Pose current_value) {
  tf::Quaternion q;
  q.setRPY(current_value.roll, current_value.pitch, current_value.yaw);

  msg_ptr->position.x = current_value.x;
  msg_ptr->position.y = current_value.y;
  msg_ptr->position.z = current_value.z;
  msg_ptr->orientation.x = q.x();
  msg_ptr->orientation.y = q.y();
  msg_ptr->orientation.z = q.z();
  msg_ptr->orientation.w = q.w();
}

void NDTMapper::setupMembers() {
  // TODO
}

void NDTMapper::getTF() {
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_baselink2primarylidar;

  double tf_x, tf_y, tf_z, tf_roll, tf_pitch, tf_yaw;
  bool received_tf = false;

  // 1. Try getting base_link -> lidar TF from TF tree
  try {
    nh_private_.getParam("lidar_frame", lidar_frame);
    tf_listener.waitForTransform("base_link", lidar_frame, ros::Time(),
                                 ros::Duration(1.0));
    tf_listener.lookupTransform("base_link", lidar_frame, ros::Time(),
                                tf_baselink2primarylidar);
  } catch (tf::TransformException &ex) {
    ROS_WARN(
        "Query base_link to primary lidar frame through TF tree failed: %s",
        ex.what());
    no_b2l_tf = true;
    received_tf = false;
  }

  // 2. Try getting from config files
  if (!received_tf) {
    if (!nh_private_.getParam("baselink2LiDAR", b2l_tf))
      ROS_ERROR(
          "Failed to get parameter from server. Please CHECK THE CONFIG FILE");
    else
      LOG(INFO) << "LOAD base2LiDAR TF success from CONFIG file, baselink to "
                   "LiDAR: [x,y,z,roll,pitch,yaw]"
                << b2l_tf[0] << b2l_tf[1] << b2l_tf[2] << b2l_tf[3] << b2l_tf[4]
                << b2l_tf[5];

    tf::Vector3 trans(b2l_tf[0], b2l_tf[1], b2l_tf[2]);
    tf::Quaternion quat;
    quat.setRPY(b2l_tf[3], b2l_tf[4], b2l_tf[5]);
    tf_baselink2primarylidar.setOrigin(trans);
    tf_baselink2primarylidar.setRotation(quat);
    received_tf = true;
  }

  if (received_tf) {
    ROS_INFO("base_link to primary lidar transform queried successfully");

    tf_x = tf_baselink2primarylidar.getOrigin().getX();
    tf_y = tf_baselink2primarylidar.getOrigin().getY();
    tf_z = tf_baselink2primarylidar.getOrigin().getZ();
    Eigen::Translation3f tl_btol(tf_x, tf_y, tf_z); // tl: translation

    tf::Matrix3x3(tf_baselink2primarylidar.getRotation())
        .getRPY(tf_roll, tf_pitch, tf_yaw);
    Eigen::AngleAxisf rot_x_btol(tf_roll,
                                 Eigen::Vector3f::UnitX()); // rot: rotation
    Eigen::AngleAxisf rot_y_btol(tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(tf_yaw, Eigen::Vector3f::UnitZ());
    tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    tf_ltob = tf_btol.inverse();
  } else
    ROS_ERROR("Failed to query base_link to primary lidar transform");
}

void NDTMapper::setConfig() {
  nh_private_.getParam("use_odom", _use_odom);
  nh_private_.getParam("use_imu", _use_imu);
  nh_private_.getParam("imu_upside_down", _imu_upside_down);
  nh_private_.getParam("odom_inverse", _odom_inverse);
  nh_private_.getParam("imu_topic", _imu_topic);
  nh_private_.getParam("lidar_topic", _lidar_topic);
  nh_private_.getParam("odom_topic", _odom_topic);
  nh_private_.getParam("debug_print", _debug_print);
  nh_private_.getParam("output_odom_topic", _output_odom_topic);
  nh_private_.getParam("odom_lidar_topic", _odom_lidar_topic);

  nh_private_.getParam("incremental_voxel_update", _incremental_voxel_update);
  nh_private_.getParam("resolution", config_.ndt_res);
  nh_private_.getParam("step_size", config_.step_size);
  nh_private_.getParam("trans_epsilon", config_.trans_eps);
  nh_private_.getParam("max_iterations", config_.max_iter);
  nh_private_.getParam("leaf_size", config_.voxel_leaf_size);
  nh_private_.getParam("min_scan_range", config_.min_scan_range);
  nh_private_.getParam("max_scan_range", config_.max_scan_range);
  nh_private_.getParam("min_add_scan_shift", config_.min_add_scan_shift);
  nh_private_.getParam("save_frame_point", config_.save_frame_point);

  std::cout << "======================================> PARAM" << std::endl;
  std::cout << "lidar_topic: " << _lidar_topic << std::endl;
  // std::cout << "method_type: " << static_cast<int>(_method_type) <<
  // std::endl;
  std::cout << "use_odom: " << _use_odom << std::endl;
  std::cout << "use_imu: " << _use_imu << std::endl;
  std::cout << "imu_upside_down: " << _imu_upside_down << std::endl;
  std::cout << "imu_topic: " << _imu_topic << std::endl;
  std::cout << "incremental_voxel_update: " << _incremental_voxel_update
            << std::endl;
  std::cout << "PARAM <====================================== " << std::endl;

  if (_debug_print)
    google::SetStderrLogging(google::INFO);
  else
    google::SetStderrLogging(google::WARNING);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
NDTMapper::filterPoints(const sensor_msgs::PointCloud2::ConstPtr &input) {
  pcl::PointCloud<pcl::PointXYZI> tmp, scan;
  pcl::PointXYZI p;
  double p_radius;
  pcl::fromROSMsg(*input, tmp);

#pragma omp parallel for num_threads(4)
  for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin();
       item != tmp.end(); item++) {
    p.x = (double)item->x;
    p.y = (double)item->y;
    p.z = (double)item->z;
    p.intensity = (double)item->intensity;

    p_radius = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (config_.min_scan_range < p_radius && p_radius < config_.max_scan_range)
      scan.push_back(p);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(
      new pcl::PointCloud<pcl::PointXYZI>(scan));
  return scan_ptr;
}

Eigen::Matrix4f NDTMapper::calTranslation(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr) {
  TIC;
  // ===========================> NDT CALCULATE <===========================
  static int initial_scan_loaded = 0;
  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0) {
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
    map += *transformed_scan_ptr;
    ndt_.setTransformationEpsilon(config_.trans_eps);
    ndt_.setStepSize(config_.step_size);
    ndt_.setResolution(config_.ndt_res);
    ndt_.setMaximumIterations(config_.max_iter);
    pcl::PointCloud<pcl::PointXYZI>::Ptr inital_map_ptr(
        new pcl::PointCloud<pcl::PointXYZI>(map));
    ndt_.setInputTarget(inital_map_ptr); // after first it will use the last one
    initial_scan_loaded = 1;
  }

  guess_lidar_pose = previous_pose + diff_pose;
  // TODO odom imu and filter TODO
  Pose guess_pose = guess_lidar_pose;

  Eigen::Matrix4f init_guess = Pose2Matrix(guess_pose) * tf_btol;

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  // IMPORTANT STEP HERE
  ndt_.setInputSource(filtered_scan_ptr);
  ndt_.align(*output_cloud, init_guess);

  fitness_score = ndt_.getFitnessScore();
  Eigen::Matrix4f t_localizer = ndt_.getFinalTransformation();
  has_converged = ndt_.hasConverged();
  final_num_iteration = ndt_.getFinalNumIteration();
  TOC("NDT CALCULATION", _debug_print);
  return t_localizer;
}

bool NDTMapper::saveMap(lidar_localizer_msg::SaveMap::Request &req,
                        lidar_localizer_msg::SaveMap::Response &res) {
  std::cout
      << "-----------------------------------------------------------------"
      << std::endl;
  TIC;
  std::string save_map_path = req.path;
  double save_map_filter_res = req.filter_res;
  std::cout << " ==========> ATTENTION! START SAVING MAP" << std::endl;
  std::cout << "filter resolution: " << save_map_filter_res << std::endl;
  std::cout << "save map path:     " << save_map_path << std::endl;

  lockPointCloud.lock();
  // Writing Point Cloud data to PCD file
  auto map_ptr = map.makeShared();
  if (save_map_filter_res == 0.0) {
    pcl::io::savePCDFileASCII(save_map_path, map);
    std::cout << "Saved " << map.points.size() << " data points to "
              << save_map_path << "." << std::endl;
  } else {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(save_map_filter_res, save_map_filter_res,
                                  save_map_filter_res);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*map_filtered);
    pcl::io::savePCDFileASCII(save_map_path, *map_filtered);
    std::cout << "Original: " << map.points.size() << " points." << std::endl;
    std::cout << "Filtered: " << map_filtered->points.size() << " points."
              << std::endl;
  }
  lockPointCloud.unlock();
  std::cout << " ===========> SAVE MAP SUCCESS, PLEASE CHECK:" << save_map_path
            << std::endl;
  TOC("SAVE MAP", true);
  std::cout
      << "-----------------------------------------------------------------"
      << std::endl;
  res.success = true;
  return res.success;
}

void NDTMapper::pubOtherMsgs(const sensor_msgs::PointCloud2::ConstPtr &input) {
  diff_pose = current_pose - previous_pose;
  previous_pose = current_pose;

  geometry_msgs::PoseStamped current_pose_msg, guess_pose_msg;
  current_pose_msg.header.frame_id = "map";
  current_pose_msg.header.stamp = current_scan_time;
  Pose2MsgPose(&(current_pose_msg.pose), current_pose);

  current_pose_pub.publish(current_pose_msg);
  current_lidar_pub.publish(input);

  nav_msgs::Odometry odom_to_pub;
  odom_to_pub.header.stamp = current_scan_time;
  odom_to_pub.header.frame_id = "map";
  odom_to_pub.child_frame_id = lidar_frame;
  Pose2MsgPose(&(odom_to_pub.pose.pose), current_pose);

  // time to cal vel
  scan_duration = current_scan_time - previous_scan_time;
  double secs = scan_duration.toSec();
  previous_scan_time.sec = current_scan_time.sec;
  previous_scan_time.nsec = current_scan_time.nsec;

  double current_velocity_x = diff_pose.x / secs;
  double current_velocity_y = diff_pose.y / secs;
  double current_velocity_z = diff_pose.z / secs;
  odom_to_pub.twist.twist.linear.x = current_velocity_x;
  odom_to_pub.twist.twist.linear.y = current_velocity_y;
  odom_to_pub.twist.twist.linear.z = current_velocity_z;

  current_odom_pub.publish(odom_to_pub);

  /*** if path is too large, the rvis will crash ***/
  static int jjj = 0;
  jjj++;
  path_buffer.header = current_pose_msg.header;
  if (jjj % 5 == 0) {
    path_buffer.poses.push_back(current_pose_msg);
    path_viz_pub.publish(path_buffer);
  }
}
} // namespace ndt_mapping