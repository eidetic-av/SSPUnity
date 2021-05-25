//
// Created by eidetic-av on 07-05-2021
//

#include "kinect2_reader.h"

#include <fstream>
#include <iostream>
#include <string>

Kinect2Reader::Kinect2Reader(uint8_t _device_index) {
  device_index_ = _device_index;

  stream_color_ = true;
  stream_depth_ = true;
  stream_joints_ = false;

  stream_id_ = RandomString(16);

  // TODO exit if no devices present
  // probably using FAILED or CHECK

  GetDefaultKinectSensor(&sensor_);
  sensor_->get_CoordinateMapper(&mapper_);
  sensor_->Open();
  sensor_->OpenMultiSourceFrameReader(
      FrameSourceTypes::FrameSourceTypes_Depth |
          FrameSourceTypes::FrameSourceTypes_Color,
      &reader_);

  spdlog::info("Opened Kinect 2 device");

  // timeout_ms_ = 1000 / camera_fps;
  timeout_ms_ = 1000 / 30;

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

Kinect2Reader::~Kinect2Reader() {}

IMultiSourceFrame *frame = NULL;
bool exported_lut = false;

void Kinect2Reader::NextFrame() {

  // TODO export the lut file and quit
  // if (exported_lut)
  //   return;

  // unsigned int table_size;
  // PointF *table;
  // spdlog::info("going to export");
  // if (SUCCEEDED(
  //         mapper_->GetDepthFrameToCameraSpaceTable(&table_size, &table))) {

  //   spdlog::info("exporting  lut with table size: {}", table_size);

  //   std::ofstream output_file;

  //   output_file.open("KINECT2.txt");

  //   for (int i = 0; i < table_size; i++) {
  //     PointF scalar = table[i];
  //     output_file << scalar.X << "," << -scalar.Y << " ";
  //     std::cout << ".";
  //   }

  //   output_file.close();

  //   CoTaskMemFree(table);
  //   exported_lut = true;
  // }
  // spdlog::info("done!");
  // return;
  // end TODO

  current_frame_.clear();
  uint64_t capture_timestamp = CurrentTimeMs();

  // get depth data
  IDepthFrameReference *depth_frame_ref = NULL;
  frame->get_DepthFrameReference(&depth_frame_ref);

  IDepthFrame *k2_depth_frame = NULL;
  if (SUCCEEDED(depth_frame_ref->AcquireFrame(&k2_depth_frame))) {
    k2_depth_frame->CopyFrameDataToArray(depth_width * depth_height,
                                         depth_buffer);

    // create and queue the frame for the depth image
    std::shared_ptr<FrameStruct> depth_frame =
        std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
    depth_frame->sensor_id = 1;
    depth_frame->frame_type = 1;
    depth_frame->frame_data_type = 3;
    depth_frame->frame_id = frame_counter_.at(1)++;
    depth_frame->timestamps.push_back(capture_timestamp);
    depth_frame->timestamps.push_back(capture_timestamp);

    unsigned int depth_frame_size = sizeof(depth_buffer);
    depth_frame->frame.resize(depth_frame_size + 2 * sizeof(int));

    memcpy(&depth_frame->frame[0], &depth_width, sizeof(int));
    memcpy(&depth_frame->frame[4], &depth_height, sizeof(int));
    memcpy(&depth_frame->frame[8], depth_buffer, depth_frame_size);

    // put decoded frames into our current_frame_ array
    current_frame_.push_back(depth_frame);

    // Write the depth 2 rgb map
    mapper_->MapDepthFrameToColorSpace(depth_width * depth_height, depth_buffer,
                                       depth_width * depth_height, depth2color);

    // get the colour frame from the reader,
    IColorFrameReference *color_frame_ref = NULL;
    frame->get_ColorFrameReference(&color_frame_ref);

    IColorFrame *k2_color_frame = NULL;
    if (SUCCEEDED(color_frame_ref->AcquireFrame(&k2_color_frame))) {
      k2_color_frame->CopyConvertedFrameDataToArray(
          color_width * color_height * 4, color_buffer, ColorImageFormat_Bgra);

      // transform the color_buffer into depth space
      int dest = 0;
      for (int i = 0; i < depth_width * depth_height; i++) {
        ColorSpacePoint p = depth2color[i];
        int color_idx = (int)p.X + color_width * (int)p.Y;
        // If colour pixel is out of bounds, fill with black
        if (p.X < 0 || p.Y < 0 || p.X > color_width || p.Y > color_height) {
          transformed_color_buffer[dest++] = 0;
          transformed_color_buffer[dest++] = 0;
          transformed_color_buffer[dest++] = 0;
          transformed_color_buffer[dest++] = 0;
        } else {
          // otherwise fill the appropriate colour
          transformed_color_buffer[dest++] = color_buffer[(4 * color_idx) + 0];
          transformed_color_buffer[dest++] = color_buffer[(4 * color_idx) + 1];
          transformed_color_buffer[dest++] = color_buffer[(4 * color_idx) + 2];
          transformed_color_buffer[dest++] = color_buffer[(4 * color_idx) + 3];
        }
      }

      // create and queue the frame for the color image
      std::shared_ptr<FrameStruct> color_frame =
          std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
      color_frame->sensor_id = 0;
      color_frame->frame_type = 0;
      color_frame->frame_data_type = 2;
      color_frame->frame_id = frame_counter_.at(0)++;
      color_frame->timestamps.push_back(capture_timestamp);
      color_frame->timestamps.push_back(capture_timestamp);

      unsigned int transformed_color_frame_size =
          sizeof(transformed_color_buffer);
      color_frame->frame.resize(transformed_color_frame_size + 2 * sizeof(int));

      memcpy(&color_frame->frame[0], &depth_width, sizeof(int));
      memcpy(&color_frame->frame[4], &depth_height, sizeof(int));
      memcpy(&color_frame->frame[8], transformed_color_buffer,
             transformed_color_frame_size);

      current_frame_.push_back(color_frame);
    }

    // Release colour camera resources
    if (color_frame_ref)
      color_frame_ref->Release();
    if (k2_color_frame)
      k2_color_frame->Release();
  }

  // release depth resources
  if (depth_frame_ref)
    depth_frame_ref->Release();
  if (k2_depth_frame)
    k2_depth_frame->Release();

  // release all resources
  if (frame) {
    frame->Release();
    frame = NULL;
  }
}

bool Kinect2Reader::HasNextFrame() {
  return SUCCEEDED(reader_->AcquireLatestFrame(&frame));
}

void Kinect2Reader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> Kinect2Reader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int Kinect2Reader::GetFps() { return 30; }

std::vector<unsigned int> Kinect2Reader::GetType() {
  std::vector<unsigned int> types;

  if (stream_color_) {
    types.push_back(0);
  }
  if (stream_depth_) {
    types.push_back(1);
  }
  if (stream_joints_) {
    types.push_back(2);
  }

  return types;
}

void Kinect2Reader::GoToFrame(unsigned int frame_id) {}
unsigned int Kinect2Reader::GetCurrentFrameId() { return frame_counter_.at(0); }
