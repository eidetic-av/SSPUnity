//
// Created by eidetic-av on 07-05-2021
//

#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <vector>

#include <utils/logger.h>

#include <cereal/archives/binary.hpp>

#include <readers/ireader.h>
#include <structs/frame_struct.hpp>
#include <utils/image_decoder.h>
#include <utils/kinect_utils.h>
#include <utils/video_utils.h>

#include <Kinect.h>

extern std::atomic_bool exiting;

// TODO call k4a_device_close on every failed CHECK
// #define CHECK(x, device) \
//   { \
//     auto retval = (x); \
//     if (retval) { \
//       spdlog::error("\"Runtime error: {} returned {} ", #x, retval); \
//       k4a_device_close(device); \
//       exit(1); \
//     } \
//   }
#define CHECK(x, device)                                                       \
  {}

class Kinect2Reader : public IReader {
private:
  static const int depth_width = 512;
  static const int depth_height = 424;
  static const int color_width = 1920;
  static const int color_height = 1080;

  unsigned short depth_buffer[depth_width * depth_height];
  unsigned char color_buffer[color_width * color_height * 4];
  unsigned char transformed_color_buffer[depth_width * depth_height * 4];

  ColorSpacePoint depth2color[depth_width * depth_height];
  CameraSpacePoint depth2xyz[depth_width * depth_height];

  std::vector<unsigned int> frame_counter_;

  bool stream_color_;
  bool stream_depth_;
  bool stream_joints_;

  std::string stream_id_;

  uint8_t device_index_;
  int32_t timeout_ms_;

  IKinectSensor *sensor_;
  IMultiSourceFrameReader *reader_;

  ICoordinateMapper *mapper_;

  FrameStruct frame_template_;
  std::vector<std::shared_ptr<CodecParamsStruct>> codec_params_structs_;
  std::shared_ptr<CameraCalibrationStruct> camera_calibration_struct_;

  std::vector<std::shared_ptr<FrameStruct>> current_frame_;

public:
  Kinect2Reader(uint8_t device_index);

  ~Kinect2Reader();

  void Reset();

  bool HasNextFrame();

  void NextFrame();

  std::vector<std::shared_ptr<FrameStruct>> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

  virtual void GoToFrame(unsigned int frame_id);

  unsigned int GetFps();

  std::vector<unsigned int> GetType();
};
