#ifndef NDT_MAPPER_H
#define NDT_MAPPER_H

#include <fstream>
#include <glog/logging.h>
#include <iostream>
#include <math.h>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_localizer_msg/SaveMap.h"
#include "ndt_cpu/NormalDistributionsTransform.h"
#include "pcl_omp_registration/ndt.h"
#include "ground_segmentation/ground_segmentation.h"

namespace ndt_mapping {

class Pose {
public:
  Pose() { x = y = z = roll = pitch = yaw = 0.0; }
  virtual ~Pose() = default;
  void setPose(const Eigen::Matrix4f &t, const tf::Matrix3x3 &m) {
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    m.getRPY(roll, pitch, yaw, 1);
  }

  double calDistance() {
    double dis = sqrt(x * x + y * y + z * z);
    return dis;
  }
  void operator=(const Pose &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    roll = p.roll;
    pitch = p.pitch;
    yaw = p.yaw;
  }

  Pose operator+(const Pose &p) const {
    Pose res;
    res.x = x + p.x;
    res.y = y + p.y;
    res.z = z + p.z;
    res.roll = roll + p.roll;
    res.pitch = pitch + p.pitch;
    res.yaw = yaw + p.yaw;
    return res;
  }

  Pose operator-(const Pose &p) const {
    Pose res;
    res.x = x - p.x;
    res.y = y - p.y;
    res.z = z - p.z;

    double diff_rad = yaw - p.yaw;
    if (diff_rad >= M_PI)
      diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
      diff_rad = diff_rad + 2 * M_PI;
    res.yaw = diff_rad;

    // TODO ? is necessary ?
    res.roll = 0;
    res.pitch = 0;

    return res;
  }

  friend inline std::ostream &operator<<(std::ostream &os, const Pose &p) {
    os << std::fixed << std::setprecision(2) << " Position: (x:" << p.x
       << ") (y:" << p.y << ") (z:" << p.z << "); "
       << "Rotation: (roll:" << p.roll << ") (pitch:" << p.pitch
       << ") (yaw:" << p.yaw << ")\n";
    return os;
  }

  double x, y, z, roll, pitch, yaw;
};

class NDTMapper {
public:
  struct Config {
    int ros_spinner_threads = std::thread::hardware_concurrency();

    int save_frame_point = 10;

    // default param
    int max_iter = 30;            // Maximum iterations
    float ndt_res = 1.0;          // Resolution
    double step_size = 0.1;       // Step size
    double trans_eps = 0.01;      // Transformation epsilon
    double voxel_leaf_size = 2.0; // Leaf size of VoxelGrid filter.
    double min_scan_range = 5.0;
    double max_scan_range = 200.0;
    double min_add_scan_shift = 1.0;

    // default param for PMF
    int max_window_size = 1;
    float slope = 1.0;
    float initial_distance = 0.5;
    float max_distance = 2.0;
    float cell_size = 0.5; // inside function default is 1
    float base_b = 1.0; // inside function default is 2.0
  };

  NDTMapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~NDTMapper() = default;

  // ROS callbacks.
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input);
  void odom_callback(const nav_msgs::Odometry::ConstPtr &input);
  void imu_callback(const sensor_msgs::Imu::Ptr &input);

  // IO.
  const Config &getConfig() const { return config_; }

  template <typename Type>
  void Pose2MsgPose(Type msg_ptr, const Pose current_value);
  Pose MsgPose2Pose(const nav_msgs::Odometry::ConstPtr &input);
  Eigen::Matrix4f Pose2Matrix(const Pose &p);

  tf::Matrix3x3 setValue(const Eigen::Matrix4f &t) {
    tf::Matrix3x3 mat;
    mat.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)),
                 static_cast<double>(t(0, 2)), static_cast<double>(t(1, 0)),
                 static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
                 static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)),
                 static_cast<double>(t(2, 2)));
    return mat;
  }
  
  // Useful function
  template <typename PointT>
  void removeFloor(const typename pcl::PointCloud<PointT>::Ptr in_cloud_ptr,
                   typename pcl::PointCloud<PointT>::Ptr out_nofloor_cloud_ptr,
                   typename pcl::PointCloud<PointT>::Ptr out_onlyfloor_cloud_ptr);

  // METHOD CHOOSE TODO
  cpu::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
  // pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;
  // pcl_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt_;

private:
  // Setup.
  void setupMembers();
  void setupRos();
  void setConfig();
  void getTF();

  // function
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  filterPoints(const sensor_msgs::PointCloud2::ConstPtr &input);
  Eigen::Matrix4f
  calTranslation(const pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr,
                 pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr);
  bool saveMap(lidar_localizer_msg::SaveMap::Request &req,
               lidar_localizer_msg::SaveMap::Response &res);
  void pubOtherMsgs(const sensor_msgs::PointCloud2::ConstPtr &input);

  // variables
  Config config_;

  // things we'd like to calculate
  Pose previous_pose, current_pose, added_pose;
  Pose imu_pose, odom_pose, guess_lidar_pose, guess_base_pose;
  Pose diff_pose, offset_imu, offset_odom;

  Eigen::Matrix4f tf_btol, tf_ltob;

  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers, Publishers, Services, Timers.
  ros::Publisher current_odom_pub, offset_odom_pub;
  ros::Publisher current_pose_pub, path_viz_pub;
  ros::Publisher ndt_map_pub, current_lidar_pub, ground_pt_pub;
  ros::Subscriber points_sub, odom_sub, imu_sub;

  ros::ServiceServer save_map_srv;

  ros::Time current_scan_time;
  ros::Time previous_scan_time;
  ros::Duration scan_duration;

  tf::TransformBroadcaster br_;
  pcl::PointCloud<pcl::PointXYZI> map;

  nav_msgs::Path path_buffer;
  std::vector<float> b2l_tf;
  std::string lidar_frame;

  bool no_b2l_tf = false;
  bool _use_imu = false;
  bool _use_odom = false;
  bool _imu_upside_down = false;
  bool _odom_inverse = false;
  bool _debug_print = true;
  bool _incremental_voxel_update = false;
  bool _process_ground_cloud = false;
  bool _pub_ground_cloud = false;

  std::string _imu_topic = "/imu_raw";
  std::string _lidar_topic = "points_raw";
  std::string _odom_topic = "/vehicle/odom";
  std::string _output_odom_topic = "/guess_pose_odom";
  std::string _odom_lidar_topic = "/odom_lidar";
  std::string _output_offset_odom = "/offset_leg_odom";

  // ndt result
  double fitness_score;
  double transformation_probability;
  int final_num_iteration;
  bool has_converged;

  // save lock resource
  std::mutex lockPointCloud;
  
  // ground
  GroundSegmentation segmenter_;
};

} // namespace ndt_mapping

#endif