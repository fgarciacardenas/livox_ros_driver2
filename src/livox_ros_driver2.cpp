//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>

#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

using namespace livox_ros;

#ifdef BUILDING_ROS1
int main(int argc, char **argv) {
  /** Ros related */
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "livox_lidar_publisher");

  // ros::NodeHandle livox_node;
  livox_ros::DriverNode livox_node;

  DRIVER_INFO(livox_node, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq  = 10.0; /* Hz */
  int output_type      = kOutputToRos;
  std::string frame_id = "livox_frame";
  bool lidar_bag = true;
  bool imu_bag   = false;

  livox_node.GetNode().getParam("xfer_format", xfer_format);
  livox_node.GetNode().getParam("multi_topic", multi_topic);
  livox_node.GetNode().getParam("data_src", data_src);
  livox_node.GetNode().getParam("publish_freq", publish_freq);
  livox_node.GetNode().getParam("output_data_type", output_type);
  livox_node.GetNode().getParam("frame_id", frame_id);
  livox_node.GetNode().getParam("enable_lidar_bag", lidar_bag);
  livox_node.GetNode().getParam("enable_imu_bag", imu_bag);

  printf("data source:%u.\n", data_src);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  livox_node.future_ = livox_node.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                        publish_freq, frame_id, lidar_bag, imu_bag);
  livox_node.lddc_ptr_->SetRosNode(&livox_node);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(livox_node, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    livox_node.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node, "Invalid data src (%d), please check the launch file", data_src);
  }

  livox_node.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node);
  livox_node.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node);
  while (ros::ok()) { usleep(10000); }

  return 0;
}

#elif defined BUILDING_ROS2
namespace livox_ros
{
DriverNode::DriverNode(const rclcpp::NodeOptions & node_options)
: Node("livox_driver_node", node_options)
{
  DRIVER_INFO(*this, "Livox Ros Driver2 Version: %s", LIVOX_ROS_DRIVER2_VERSION_STRING);

  /** Init default system parameter */
  int xfer_format = kPointCloud2Msg;
  int multi_topic = 0;
  int data_src = kSourceRawLidar;
  double publish_freq = 10.0; /* Hz */
  int output_type = kOutputToRos;
  std::string frame_id;
  bool publish_pointcloud = false;
  bool crop_pointcloud = false;
  double pointcloud_min_range = 0.0;
  double pointcloud_max_range = 0.0;
  int pointcloud_qos_depth = 5;
  bool publish_scan = false;
  std::string scan_topic;
  double scan_angle_min = -3.14159265358979323846;
  double scan_angle_max = 3.14159265358979323846;
  double scan_angle_increment = 0.00872664625997164788;
  double scan_time = 0.0;
  double scan_range_min = 0.2;
  double scan_range_max = 8.0;
  double scan_min_z = -1000.0;
  double scan_max_z = 1000.0;
  bool scan_self_filter_enabled = false;
  double scan_self_filter_min_x = 0.0;
  double scan_self_filter_max_x = 0.0;
  double scan_self_filter_min_y = 0.0;
  double scan_self_filter_max_y = 0.0;
  bool scan_use_inf = true;
  int scan_qos_depth = 5;

  this->declare_parameter("xfer_format", xfer_format);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", data_src);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", output_type);
  this->declare_parameter("frame_id", "frame_default");
  this->declare_parameter("publish_pointcloud", false);
  this->declare_parameter("crop_pointcloud", false);
  this->declare_parameter("pointcloud_min_range", 0.0);
  this->declare_parameter("pointcloud_max_range", 0.0);
  this->declare_parameter("pointcloud_qos_depth", 5);
  this->declare_parameter("publish_scan", false);
  this->declare_parameter("scan_topic", "/livox/scan");
  this->declare_parameter("scan_angle_min", -3.14159265358979323846);
  this->declare_parameter("scan_angle_max", 3.14159265358979323846);
  this->declare_parameter("scan_angle_increment", 0.00872664625997164788);
  this->declare_parameter("scan_time", 0.0);
  this->declare_parameter("scan_range_min", 0.2);
  this->declare_parameter("scan_range_max", 8.0);
  this->declare_parameter("scan_min_z", -1000.0);
  this->declare_parameter("scan_max_z", 1000.0);
  this->declare_parameter("scan_self_filter_enabled", false);
  this->declare_parameter("scan_self_filter_min_x", 0.0);
  this->declare_parameter("scan_self_filter_max_x", 0.0);
  this->declare_parameter("scan_self_filter_min_y", 0.0);
  this->declare_parameter("scan_self_filter_max_y", 0.0);
  this->declare_parameter("scan_use_inf", true);
  this->declare_parameter("scan_qos_depth", 5);
  this->declare_parameter("user_config_path", "path_default");
  this->declare_parameter("cmdline_input_bd_code", "000000000000001");
  this->declare_parameter("lvx_file_path", "/home/livox/livox_test.lvx");

  this->get_parameter("xfer_format", xfer_format);
  this->get_parameter("multi_topic", multi_topic);
  this->get_parameter("data_src", data_src);
  this->get_parameter("publish_freq", publish_freq);
  this->get_parameter("output_data_type", output_type);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("publish_pointcloud", publish_pointcloud);
  this->get_parameter("crop_pointcloud", crop_pointcloud);
  this->get_parameter("pointcloud_min_range", pointcloud_min_range);
  this->get_parameter("pointcloud_max_range", pointcloud_max_range);
  this->get_parameter("pointcloud_qos_depth", pointcloud_qos_depth);
  this->get_parameter("publish_scan", publish_scan);
  this->get_parameter("scan_topic", scan_topic);
  this->get_parameter("scan_angle_min", scan_angle_min);
  this->get_parameter("scan_angle_max", scan_angle_max);
  this->get_parameter("scan_angle_increment", scan_angle_increment);
  this->get_parameter("scan_time", scan_time);
  this->get_parameter("scan_range_min", scan_range_min);
  this->get_parameter("scan_range_max", scan_range_max);
  this->get_parameter("scan_min_z", scan_min_z);
  this->get_parameter("scan_max_z", scan_max_z);
  this->get_parameter("scan_self_filter_enabled", scan_self_filter_enabled);
  this->get_parameter("scan_self_filter_min_x", scan_self_filter_min_x);
  this->get_parameter("scan_self_filter_max_x", scan_self_filter_max_x);
  this->get_parameter("scan_self_filter_min_y", scan_self_filter_min_y);
  this->get_parameter("scan_self_filter_max_y", scan_self_filter_max_y);
  this->get_parameter("scan_use_inf", scan_use_inf);
  this->get_parameter("scan_qos_depth", scan_qos_depth);

  if (publish_freq > 100.0) {
    publish_freq = 100.0;
  } else if (publish_freq < 0.5) {
    publish_freq = 0.5;
  } else {
    publish_freq = publish_freq;
  }

  future_ = exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  if (pointcloud_min_range < 0.0) {
    pointcloud_min_range = 0.0;
  }
  if (scan_range_min < 0.0) {
    scan_range_min = 0.0;
  }
  if (scan_angle_increment <= 0.0) {
    scan_angle_increment = 0.00872664625997164788;
  }
  if (pointcloud_qos_depth < 1) {
    pointcloud_qos_depth = 1;
  }
  if (scan_qos_depth < 1) {
    scan_qos_depth = 1;
  }

  RangeFilterConfig range_filter;
  range_filter.enabled = crop_pointcloud;
  range_filter.min_range = pointcloud_min_range;
  range_filter.max_range = pointcloud_max_range;

  ScanConfig scan_config;
  scan_config.enabled = publish_scan;
  scan_config.topic = scan_topic;
  scan_config.angle_min = scan_angle_min;
  scan_config.angle_max = scan_angle_max;
  scan_config.angle_increment = scan_angle_increment;
  scan_config.scan_time = scan_time;
  scan_config.range_min = scan_range_min;
  scan_config.range_max = scan_range_max;
  scan_config.min_z = scan_min_z;
  scan_config.max_z = scan_max_z;
  scan_config.self_filter_enabled = scan_self_filter_enabled;
  scan_config.self_filter_min_x = scan_self_filter_min_x;
  scan_config.self_filter_max_x = scan_self_filter_max_x;
  scan_config.self_filter_min_y = scan_self_filter_min_y;
  scan_config.self_filter_max_y = scan_self_filter_max_y;
  scan_config.use_inf = scan_use_inf;
  scan_config.qos_depth = static_cast<uint32_t>(scan_qos_depth);

  lddc_ptr_ = std::make_unique<Lddc>(
      xfer_format, multi_topic, data_src, output_type, publish_freq, frame_id,
      range_filter, scan_config, publish_pointcloud,
      static_cast<uint32_t>(pointcloud_qos_depth));
  lddc_ptr_->SetRosNode(this);

  if (data_src == kSourceRawLidar) {
    DRIVER_INFO(*this, "Data Source is raw lidar.");

    std::string user_config_path;
    this->get_parameter("user_config_path", user_config_path);
    DRIVER_INFO(*this, "Config file : %s", user_config_path.c_str());

    std::string cmdline_bd_code;
    this->get_parameter("cmdline_input_bd_code", cmdline_bd_code);

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(*this, "Init lds lidar success!");
    } else {
      DRIVER_ERROR(*this, "Init lds lidar fail!");
    }
  } else {
    DRIVER_ERROR(*this, "Invalid data src (%d), please check the launch file", data_src);
  }

  pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, this);
  imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, this);
}

}  // namespace livox_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(livox_ros::DriverNode)

#endif  // defined BUILDING_ROS2


void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}





















