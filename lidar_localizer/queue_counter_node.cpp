/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 Message counter in subscriber's queue.

 Yuki KITSUKAWA
 */

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <glog/logging.h>
#include <ros/ros.h>

static int enqueue = 0;
static int dequeue = 0;
static bool _debug_print = true;
static std::string _lidar_topic = "points_raw";
static ros::Publisher ndt_map_pub;
static ros::Time current_scan_time;

static void points_callback(const sensor_msgs::PointCloud2::ConstPtr &input) {
  enqueue++;
}

static void ndt_pose_callback(const geometry_msgs::PoseStamped &msg) {
  dequeue++;
  if (_debug_print)
    LOG(INFO) << "(Processed/Input): (" << dequeue << " / " << enqueue << ")"
              << std::endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "queue_counter");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  private_nh.getParam("lidar_topic", _lidar_topic);
  private_nh.getParam("debug_print", _debug_print);

  ros::Subscriber points_sub = nh.subscribe(_lidar_topic, 10, points_callback);
  ros::Subscriber ndt_map_sub =
      nh.subscribe("/current_pose", 10, ndt_pose_callback);

  ros::spin();
  return 0;
}
