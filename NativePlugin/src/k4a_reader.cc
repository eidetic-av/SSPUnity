//
// Created by amourao on 26-06-2019.
//

#include "k4a_reader.h"

std::atomic_bool exiting(false);

K4AReader::K4AReader(uint8_t _device_index,
                           ExtendedAzureConfig _device_config, bool cull_background) {
  device_index_ = _device_index;

  device_config_ = _device_config.device_config;

  stream_color_ = _device_config.stream_color;
  stream_depth_ = _device_config.stream_depth;
  stream_ir_ = _device_config.stream_ir;

  cull_background_ = cull_background;

  absolute_exposure_value_ = _device_config.absolute_exposure_value;
  record_imu_ = false;

  stream_id_ = RandomString(16);

  const uint32_t installed_devices = k4a_device_get_installed_count();
  if (device_index_ >= installed_devices) {
    spdlog::error("Kinect Device not found.");
    exit(1);
  }

  if (K4A_FAILED(k4a_device_open(device_index_, &device_))) {
    spdlog::error("Runtime error: k4a_device_open() failed.");
    exit(1);
  }

  char serial_number_buffer[256];
  size_t serial_number_buffer_size = sizeof(serial_number_buffer);
  CHECK(k4a_device_get_serialnum(device_, serial_number_buffer,
                                 &serial_number_buffer_size),
        device_);

  k4a_hardware_version_t version_info;
  CHECK(k4a_device_get_version(device_, &version_info), device_);

  spdlog::info("Kinect Device serial number: {}", serial_number_buffer);
  spdlog::info("Kinect Device version: {}; C: {}.{}.{}; D: {}.{}.{} {}.{}; A: "
               "{}.{}.{};  ",
               (version_info.firmware_build == K4A_FIRMWARE_BUILD_RELEASE
                    ? "Rel"
                    : "Dbg"),
               version_info.rgb.major, version_info.rgb.minor,
               version_info.rgb.iteration, version_info.depth.major,
               version_info.depth.minor, version_info.depth.iteration,
               version_info.depth_sensor.major, version_info.depth_sensor.minor,
               version_info.audio.major, version_info.audio.minor,
               version_info.audio.iteration);

  uint32_t camera_fps = k4a_convert_fps_to_uint(device_config_.camera_fps);

  if (camera_fps <= 0 ||
      (device_config_.color_resolution == K4A_COLOR_RESOLUTION_OFF &&
       device_config_.depth_mode == K4A_DEPTH_MODE_OFF)) {
    spdlog::error("Either the color or depth modes must be enabled to record.");
    exit(1);
  }

  if (absolute_exposure_value_ != 0) {
    if (K4A_FAILED(k4a_device_set_color_control(
            device_, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
            K4A_COLOR_CONTROL_MODE_MANUAL, absolute_exposure_value_))) {
      spdlog::error("Runtime error: k4a_device_set_color_control() failed.");
      exit(1);
    }
  } else {
    if (K4A_FAILED(k4a_device_set_color_control(
            device_, K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
            K4A_COLOR_CONTROL_MODE_AUTO, 0))) {
      spdlog::error("Runtime error: k4a_device_set_color_control() failed.");
      exit(1);
    }
  }

  CHECK(k4a_device_start_cameras(device_, &device_config_), device_);
  if (record_imu_) {
    CHECK(k4a_device_start_imu(device_), device_);
  }

  spdlog::info("Kinect Device started");

  // Wait for the first capture before starting recording.
  int32_t timeout_sec_for_first_capture = 60;
  if (device_config_.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE) {
    timeout_sec_for_first_capture = 360;
    spdlog::warn("[subordinate mode] Waiting for signal from master");
  }
  clock_t first_capture_start = clock();
  // Wait for the first capture in a loop so Ctrl-C will still exit.
  while (!exiting && (clock() - first_capture_start) <
                         (CLOCKS_PER_SEC * timeout_sec_for_first_capture)) {
    result_ = k4a_device_get_capture(device_, &capture_, 100);
    if (result_ == K4A_WAIT_RESULT_SUCCEEDED) {
      k4a_capture_release(capture_);
      break;
    } else if (result_ == K4A_WAIT_RESULT_FAILED) {
      spdlog::error(
          "Runtime error: k4a_device_get_capture() returned error: {}",
          result_);
      exit(1);
    }
  }

  if (exiting) {
    spdlog::error("Exiting");
    k4a_device_close(device_);
    exit(0);
  } else if (result_ == K4A_WAIT_RESULT_TIMEOUT) {
    spdlog::error("Timed out waiting for first capture.");
    exit(1);
  }

  // get camera calibration data
  k4a_device_get_calibration(device_, K4A_DEPTH_MODE_NFOV_UNBINNED,
                             K4A_COLOR_RESOLUTION_720P, &device_calibration_);

  camera_calibration_struct_ =
      std::shared_ptr<CameraCalibrationStruct>(new CameraCalibrationStruct());

  size_t buffer_size = 10000;
  camera_calibration_struct_->type = 0;
  camera_calibration_struct_->data.resize(buffer_size);
  k4a_buffer_result_t output_buffer_size = k4a_device_get_raw_calibration(
      device_, (uint8_t *)camera_calibration_struct_->data.data(),
      &buffer_size);

  camera_calibration_struct_->data.resize(buffer_size);
  // camera_calibration_struct_->data.resize(buffer_size + 1);
  // camera_calibration_struct_->data[buffer_size] = '\0';
  camera_calibration_struct_->extra_data.push_back(
      _device_config.device_config.depth_mode);
  camera_calibration_struct_->extra_data.push_back(
      _device_config.device_config.color_resolution);

  // create the transformation context
  transformation_handle_ = k4a_transformation_create(&device_calibration_);

  // and the image that holds the transformation result

  // create the body tracker
  if (cull_background_) {
    tracker_ = NULL;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;

    if (K4A_FAILED(k4abt_tracker_create(&device_calibration_, tracker_config,
                                        &tracker_))) {
      spdlog::error("Failed to create body tracker.");
      return;
    } else
      spdlog::info("Initialised body tracker DNN.");
  }

  timeout_ms_ = 1000 / camera_fps;

  frame_template_.frame_id = 0;
  frame_template_.device_id = 0;

  frame_template_.message_type = 0;

  frame_template_.frame_data_type = 0;
  frame_template_.scene_desc = "kinect";
  frame_template_.stream_id = RandomString(16);

  frame_counter_.push_back(0);
  frame_counter_.push_back(0);
  frame_counter_.push_back(0);

  codec_params_structs_.push_back(nullptr);
  codec_params_structs_.push_back(nullptr);
  codec_params_structs_.push_back(nullptr);
}

K4AReader::~K4AReader() {
  k4a_device_stop_cameras(device_);

  k4a_device_close(device_);

  if (color_transformation_result_ != nullptr)
    k4a_image_release(color_transformation_result_);
}

void K4AReader::NextFrame() {
  current_frame_.clear();

  do {
    result_ = k4a_device_get_capture(device_, &capture_, timeout_ms_);

    if (result_ == K4A_WAIT_RESULT_TIMEOUT) {
      continue;
    } else if (result_ != K4A_WAIT_RESULT_SUCCEEDED) {
      spdlog::error(
          "Runtime error: k4a_device_get_capture() returned error: {}",
          result_);
      break;
    }
    uint64_t capture_timestamp = CurrentTimeMs();

    // queue the tracker for processing this new capture
    if (cull_background_)
      k4a_wait_result_t queue_capture_result =
          k4abt_tracker_enqueue_capture(tracker_, capture_, 0);

    // get the depth image
    k4a_image_t depth_image = k4a_capture_get_depth_image(capture_);
    // get the colour image
    k4a_image_t color_image = k4a_capture_get_color_image(capture_);

    if (color_image && depth_image) {
      // get the resolution
      int width = k4a_image_get_width_pixels(depth_image);
      int height = k4a_image_get_height_pixels(depth_image);

      // convert color image to depth space
      k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, width, height,
                       width * 4 * (int)sizeof(uint8_t),
                       &color_transformation_result_);

      k4a_transformation_color_image_to_depth_camera(
          transformation_handle_, depth_image, color_image,
          color_transformation_result_);

      uint8_t *color_buffer =
          k4a_image_get_buffer(color_transformation_result_);
      uint8_t *depth_buffer = k4a_image_get_buffer(depth_image);

      // read the result from the tracker
      k4abt_frame_t body_frame = NULL;
      k4a_image_t body_index_map = NULL;

      if (cull_background_) {
        k4a_wait_result_t pop_frame_result =
            k4abt_tracker_pop_result(tracker_, &body_frame, K4A_WAIT_INFINITE);
        body_index_map = k4abt_frame_get_body_index_map(body_frame);

        if (body_index_map) {
          uint8_t *body_map_buffer = k4a_image_get_buffer(body_index_map);

          // cull the background from the colour image and depth images

          // do this by setting each background pixel to min
          for (int i = 0; i < width * height; i++) {
            if (body_map_buffer[i] == K4ABT_BODY_INDEX_MAP_BACKGROUND) {
              int image_i = i * 4;
              color_buffer[image_i] = 0;
              color_buffer[image_i + 1] = 0;
              color_buffer[image_i + 2] = 0;
              color_buffer[image_i + 3] = 0;

              int depth_i = i * 2;
              depth_buffer[depth_i] = 0;
              depth_buffer[depth_i + 1] = 0;
            }
          }

          // done with the body frame data, dispose of it
          k4abt_frame_release(body_frame);
          k4a_image_release(body_index_map);
        }
      }

      // create and queue the frame for the depth image
      std::shared_ptr<FrameStruct> depth_frame =
          std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
      depth_frame->sensor_id = 1;
      depth_frame->frame_type = 1;
      depth_frame->frame_data_type = 3;
      depth_frame->frame_id = frame_counter_.at(1)++;
      depth_frame->timestamps.push_back(
          k4a_image_get_device_timestamp_usec(depth_image) / 1000);
      depth_frame->timestamps.push_back(capture_timestamp);
      depth_frame->camera_calibration_data = *camera_calibration_struct_;

      size_t depth_frame_size = k4a_image_get_size(depth_image);

      depth_frame->frame.resize(depth_frame_size + 2 * sizeof(int));

      memcpy(&depth_frame->frame[0], &width, sizeof(int));
      memcpy(&depth_frame->frame[4], &height, sizeof(int));
      memcpy(&depth_frame->frame[8], depth_buffer, depth_frame_size);

      current_frame_.push_back(depth_frame);

      // create and queue the frame for the color image
      std::shared_ptr<FrameStruct> color_frame =
          std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
      color_frame->sensor_id = 0;
      color_frame->frame_type = 0;
      color_frame->frame_id = frame_counter_.at(0)++;
      color_frame->timestamps.push_back(
          k4a_image_get_device_timestamp_usec(color_image) / 1000);
      color_frame->timestamps.push_back(capture_timestamp);

      size_t color_frame_size =
          k4a_image_get_size(color_transformation_result_);
      color_frame->frame_data_type = 2;

      color_frame->frame.resize(color_frame_size + 2 * sizeof(int));

      memcpy(&color_frame->frame[0], &width, sizeof(int));
      memcpy(&color_frame->frame[4], &height, sizeof(int));
      memcpy(&color_frame->frame[8], color_buffer, color_frame_size);

      current_frame_.push_back(color_frame);
    }

    if (color_transformation_result_ != nullptr)
      k4a_image_release(color_transformation_result_);
    if (color_image != nullptr)
      k4a_image_release(color_image);
    if (depth_image != nullptr)
      k4a_image_release(depth_image);

    k4a_capture_release(capture_);
  } while (result_ == K4A_WAIT_RESULT_FAILED);
}

bool K4AReader::HasNextFrame() { return true; }

void K4AReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> K4AReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int K4AReader::GetFps() {
  if (device_config_.camera_fps == K4A_FRAMES_PER_SECOND_5)
    return 5;
  if (device_config_.camera_fps == K4A_FRAMES_PER_SECOND_15)
    return 15;
  if (device_config_.camera_fps == K4A_FRAMES_PER_SECOND_30)
    return 30;
  return -1;
}

std::vector<unsigned int> K4AReader::GetType() {
  std::vector<unsigned int> types;

  if (stream_color_) {
    types.push_back(0);
  }
  if (stream_depth_) {
    types.push_back(1);
  }
  if (stream_ir_) {
    types.push_back(2);
  }

  return types;
}

void K4AReader::GoToFrame(unsigned int frame_id) {}
unsigned int K4AReader::GetCurrentFrameId() { return frame_counter_.at(0); }
