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

#include "rm_camera_driver/hik_camera.hpp"
// std
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <memory>
#include <thread>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::camera_driver {

HikCameraNode::HikCameraNode(const rclcpp::NodeOptions & options)
: Node("camera_driver", options)
{
  FYT_REGISTER_LOGGER("camera_driver", "~/fyt2024-log", INFO);
  FYT_INFO("camera_driver", "Starting HikCameraNode!");

  // Serial number of the camera to open. Empty means "use the first enumerated device".
  camera_sn_ = this->declare_parameter("camera_sn", "");

  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);

  // Load camera info
  camera_name_ = this->declare_parameter("camera_name", "hikvision");
  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
  camera_info_url_ =
    this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");
  if (camera_info_manager_->validateURL(camera_info_url_)) {
    camera_info_manager_->loadCameraInfo(camera_info_url_);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
  } else {
    FYT_WARN("camera_driver", "Invalid camera info URL: {}", camera_info_url_);
  }

  params_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

  image_msg_.header.frame_id = "camera_optical_frame";
  image_msg_.encoding = "rgb8";

  // Watchdog / reconnect timer: tries to (re)open the camera and restarts
  // grabbing when frames stop arriving.
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000), std::bind(&HikCameraNode::timerCallback, this));

  // NOTE: the constructor never blocks on camera enumeration. The timer opens
  // the device asynchronously, so a missing camera won't stall bringup.
  //
  // Capture thread: grabs raw frames from SDK and immediately returns buffers
  // to the SDK pool (within ~1 ms).  Convert + publish are offloaded to the
  // convert thread so that buffer-pool exhaustion can never happen.
  capture_thread_ = std::thread{[this]() -> void {
    MV_FRAME_OUT out_frame;

    while (rclcpp::ok()) {
      // -------- reopen path (only executed by the capture thread) --------
      if (need_reopen_.load()) {
        FYT_INFO("camera_driver", "Capture thread handling reopen request...");
        closeDevice(/*force=*/true);
        // Give the USB stack a moment to settle after the forced close.
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        openDevice();
        need_reopen_ = false;
        continue;
      }

      if (!device_open_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      memset(&out_frame, 0, sizeof(out_frame));
      int ret;
      {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        ret = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
      }

      if (ret == MV_OK) {
        const auto stamp = this->now();

        // Copy the raw payload out of the SDK buffer, then free it back
        // to the pool *immediately* — well before conversion & publishing.
        RawFrame raw;
        raw.copyFrom(out_frame);
        raw.stamp = stamp;

        {
          std::lock_guard<std::mutex> lock(camera_mutex_);
          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
        }

        // Hand off to the convert thread.
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (frame_queue_.size() < 4) {  // drop oldest frames, not newest
            frame_queue_.push(std::move(raw));
          }
        }
        queue_cv_.notify_one();

        fail_count_ = 0;
      } else {
        if (ret != static_cast<int>(MV_E_NODATA)) {
          fail_count_++;
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 3000,
            "Get buffer failed! nRet: [%#x], fail count: %d", ret, fail_count_.load());
        }
      }
    }
  }};

  // Convert thread: pixel conversion + ROS publish.
  convert_running_ = true;
  convert_thread_ = std::thread{&HikCameraNode::convertThreadLoop, this};
}

HikCameraNode::~HikCameraNode()
{
  // Stop convert thread first so it won't try to use camera_handle_ while
  // closeDevice() tears everything down.
  if (convert_running_) {
    convert_running_ = false;
    queue_cv_.notify_all();
  }
  if (convert_thread_.joinable()) {
    convert_thread_.join();
  }
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (recorder_ != nullptr) {
    recorder_->stop();
    FYT_INFO(
      "camera_driver", "Recorder stopped! Video file {} has been saved", recorder_->path.string());
  }
  closeDevice();
  FYT_INFO("camera_driver", "HikCameraNode destroyed!");
}

bool HikCameraNode::openDevice()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);

  if (device_open_.load()) {
    return true;
  }

  MV_CC_DEVICE_INFO_LIST device_list;
  memset(&device_list, 0, sizeof(device_list));
  int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
  if (ret != MV_OK || device_list.nDeviceNum == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "No camera found! Enum ret: [%#x], count: %u", ret, device_list.nDeviceNum);
    return false;
  }
  FYT_INFO("camera_driver", "Found camera count = {}", device_list.nDeviceNum);

  // Select device by serial number, fall back to the first one
  MV_CC_DEVICE_INFO * device_info = nullptr;
  if (!camera_sn_.empty()) {
    for (unsigned int i = 0; i < device_list.nDeviceNum; ++i) {
      auto * info = device_list.pDeviceInfo[i];
      if (
        info->nTLayerType == MV_USB_DEVICE &&
        camera_sn_ == reinterpret_cast<const char *>(
                        info->SpecialInfo.stUsb3VInfo.chSerialNumber)) {
        device_info = info;
        break;
      }
    }
    if (device_info == nullptr) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Camera with SN '%s' not found!", camera_sn_.c_str());
      return false;
    }
  } else {
    device_info = device_list.pDeviceInfo[0];
  }

  ret = MV_CC_CreateHandle(&camera_handle_, device_info);
  if (ret != MV_OK) {
    FYT_ERROR("camera_driver", "Create handle failed! nRet: [{:#x}]", ret);
    camera_handle_ = nullptr;
    return false;
  }

  ret = MV_CC_OpenDevice(camera_handle_);
  if (ret != MV_OK) {
    FYT_ERROR("camera_driver", "Open device failed! nRet: [{:#x}]", ret);
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;
    return false;
  }

  // Declare exposure/gain parameters now that the device is open
  // (their range is queried from the camera).
  declareParameters();
  applyParameters();

  // Start grabbing with continuous (free-run) trigger mode
  MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
  ret = MV_CC_StartGrabbing(camera_handle_);
  if (ret != MV_OK) {
    FYT_ERROR("camera_driver", "Start grabbing failed! nRet: [{:#x}]", ret);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
    camera_handle_ = nullptr;
    return false;
  }

  // Pre-allocate the image buffer based on the actual frame size
  MV_IMAGE_BASIC_INFO img_info;
  if (MV_CC_GetImageInfo(camera_handle_, &img_info) == MV_OK) {
    image_msg_.data.resize(
      static_cast<size_t>(img_info.nHeightMax) * img_info.nWidthMax * 3);
  }

  // Start recorder if requested
  bool enable_recorder = this->get_parameter("recording").as_bool();
  int frame_rate = this->get_parameter("frame_rate").as_int();
  if (enable_recorder && recorder_ == nullptr) {
    const char * home_env = std::getenv("HOME");
    std::string home = home_env != nullptr ? home_env : ".";

    namespace fs = std::filesystem;
    std::filesystem::path video_path =
      fs::path(home) / "fyt2024-log/video/" / std::string(std::to_string(std::time(nullptr)) + ".avi");

    MVCC_INTVALUE width, height;
    MV_CC_GetIntValue(camera_handle_, "Width", &width);
    MV_CC_GetIntValue(camera_handle_, "Height", &height);
    recorder_ = std::make_unique<Recorder>(
      video_path, frame_rate, cv::Size(width.nCurValue, height.nCurValue));
    recorder_->start();
    FYT_INFO("camera_driver", "Recorder started! Video file: {}", video_path.string());
  }

  last_frame_time_ns_.store(this->now().nanoseconds());
  fail_count_ = 0;
  device_open_ = true;
  FYT_INFO("camera_driver", "Camera opened and grabbing started!");
  return true;
}

void HikCameraNode::closeDevice(bool force)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);

  device_open_ = false;
  if (recorder_ != nullptr) {
    recorder_->stop();
    recorder_.reset();
    FYT_INFO("camera_driver", "Recorder stopped!");
  }
  if (camera_handle_ != nullptr) {
    if (force) {
      // Skip both StopGrabbing and CloseDevice when the USB link is
      // suspected dead — either call can hang indefinitely as the SDK
      // tries to communicate with absent hardware.  DestroyHandle
      // performs an internal teardown that does not touch the hardware.
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
    } else {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
      camera_handle_ = nullptr;
    }
  }
  fail_count_ = 0;
}

void HikCameraNode::timerCallback()
{
  // (Re)open the device if needed
  if (!device_open_.load()) {
    openDevice();
    return;
  }

  // Frame watchdog: no frame for too long -> ask capture thread to reopen.
  // We must NOT call closeDevice() / MV_CC_StopGrabbing from this timer
  // because MVS SDK forbids cross-thread StopGrabbing+GetImageBuffer pairs.
  // We also must NOT call MV_CC_IsDeviceConnected here — it can hang
  // indefinitely during USB streaming, freezing the entire executor.
  const int64_t last_ns = last_frame_time_ns_.load();
  const double dt = (this->now().nanoseconds() - last_ns) / 1e9;
  if (last_ns != 0 && dt > 5.0) {
    FYT_WARN("camera_driver", "Camera is not alive! lost frame for {:.2f} seconds", dt);
    need_reopen_ = true;
  }
}

void HikCameraNode::convertThreadLoop()
{
  MV_CC_PIXEL_CONVERT_PARAM local_convert_param;
  memset(&local_convert_param, 0, sizeof(local_convert_param));

  while (convert_running_) {
    RawFrame raw;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] {
        return !frame_queue_.empty() || !convert_running_;
      });
      if (!convert_running_) break;
      raw = std::move(frame_queue_.front());
      frame_queue_.pop();
    }

    if (!device_open_.load()) continue;

    // Pixel conversion
    image_msg_.header.stamp = raw.stamp;
    image_msg_.height = raw.height;
    image_msg_.width = raw.width;
    image_msg_.step = raw.width * 3;
    image_msg_.data.resize(static_cast<size_t>(raw.width) * raw.height * 3);

    local_convert_param.nWidth = raw.width;
    local_convert_param.nHeight = raw.height;
    local_convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    local_convert_param.pDstBuffer = image_msg_.data.data();
    local_convert_param.nDstBufferSize = image_msg_.data.size();
    local_convert_param.pSrcData = raw.data.data();
    local_convert_param.nSrcDataLen = raw.frameLen;
    local_convert_param.enSrcPixelType = raw.srcPixelType;

    int convert_ret;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      convert_ret = MV_CC_ConvertPixelType(camera_handle_, &local_convert_param);
    }

    if (convert_ret == MV_OK) {
      camera_info_msg_.header = image_msg_.header;
      camera_pub_.publish(image_msg_, camera_info_msg_);
      if (recorder_ != nullptr) {
        recorder_->addFrame(image_msg_.data);
      }
      last_frame_time_ns_.store(rclcpp::Time(image_msg_.header.stamp).nanoseconds());
    } else {
      FYT_WARN("camera_driver", "Convert pixel failed! nRet: [{:#x}]", convert_ret);
    }
  }
}

void HikCameraNode::declareParameters()
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  // Recording options (declared here so they exist even before the camera opens)
  if (!this->has_parameter("recording")) {
    this->declare_parameter("recording", false);
  }
  if (!this->has_parameter("frame_rate")) {
    this->declare_parameter("frame_rate", 30);
  }

  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  param_desc.integer_range[0].from_value = 1;
  param_desc.integer_range[0].to_value = 200000;
  int exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
  FYT_INFO("camera_driver", "Exposure time: {}", exposure_time);

  // Gain
  param_desc.description = "Gain";
  if (MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value) == MV_OK) {
    param_desc.integer_range[0].from_value = static_cast<int64_t>(f_value.fMin);
    param_desc.integer_range[0].to_value = static_cast<int64_t>(f_value.fMax);
    double gain = this->declare_parameter("gain", static_cast<double>(f_value.fCurValue), param_desc);
    FYT_INFO("camera_driver", "Gain: {}", gain);
  } else {
    this->declare_parameter("gain", 0.0, param_desc);
  }
}

void HikCameraNode::applyParameters()
{
  // Caller must hold camera_mutex_
  int exposure_time = this->get_parameter("exposure_time").as_int();
  int ret = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(exposure_time));
  if (ret != MV_OK) {
    FYT_WARN("camera_driver", "Set exposure time failed! nRet: [{:#x}]", ret);
  }

  double gain = this->get_parameter("gain").as_double();
  ret = MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(gain));
  if (ret != MV_OK) {
    FYT_WARN("camera_driver", "Set gain failed! nRet: [{:#x}]", ret);
  }
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "exposure_time") {
      // Accept both int and double to avoid ParameterTypeException
      double value = param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                       ? static_cast<double>(param.as_int())
                       : param.as_double();
      int status = MV_OK;
      if (device_open_.load()) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", static_cast<float>(value));
      }
      if (MV_OK != status) {
        result.successful = false;
        result.reason = "Failed to set exposure time, status = " + std::to_string(status);
      }
    } else if (param.get_name() == "gain") {
      double value = param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                       ? static_cast<double>(param.as_int())
                       : param.as_double();
      int status = MV_OK;
      if (device_open_.load()) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        status = MV_CC_SetFloatValue(camera_handle_, "Gain", static_cast<float>(value));
      }
      if (MV_OK != status) {
        result.successful = false;
        result.reason = "Failed to set gain, status = " + std::to_string(status);
      }
    } else if (
      param.get_name() == "recording" ||
      param.get_name() == "frame_rate" ||
      param.get_name() == "camera_info_url" ||
      param.get_name() == "camera_name" ||
      param.get_name() == "use_sensor_data_qos" ||
      param.get_name() == "camera_sn") {
      // These parameters are only used at startup / reopen; accept silently.
    } else {
      result.successful = false;
      result.reason = "Unknown parameter: " + param.get_name();
    }
  }
  return result;
}

}  // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::HikCameraNode)
