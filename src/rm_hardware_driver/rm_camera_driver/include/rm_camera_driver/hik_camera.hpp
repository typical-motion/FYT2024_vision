// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RM_CAMERA_DRIVER__HIK_CAMERA_HPP_
#define RM_CAMERA_DRIVER__HIK_CAMERA_HPP_

// Hikvision Camera SDK
#include "hikvision/MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// std
#include <atomic>
#include <memory>
#include <mutex>
// project
#include "rm_camera_driver/recorder.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::camera_driver {

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options);
  ~HikCameraNode() override;

private:
  // Try to enumerate and open the camera, then start grabbing.
  // Returns true on success. Safe to call repeatedly (no-op if already open).
  bool openDevice();
  // Stop grabbing and release the camera handle. Safe to call when not open.
  void closeDevice();

  void declareParameters();
  void applyParameters();
  void timerCallback();

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<Recorder> recorder_;

  void * camera_handle_ = nullptr;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_sn_;
  std::string camera_name_;
  std::string camera_info_url_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  // Protects camera_handle_ and all MVS SDK calls (SDK is NOT thread-safe)
  std::mutex camera_mutex_;
  std::atomic<bool> device_open_{false};
  std::atomic<int> fail_count_{0};
  // Nanoseconds timestamp of the last received frame (RCL_ROS_TIME epoch).
  // Updated by the capture thread, read by the watchdog timer.
  std::atomic<int64_t> last_frame_time_ns_{0};
  // Set by the watchdog timer to request the capture thread to close + reopen.
  std::atomic<bool> need_reopen_{false};

  std::thread capture_thread_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

}  // namespace fyt::camera_driver

#endif  // RM_CAMERA_DRIVER__HIK_CAMERA_HPP_
