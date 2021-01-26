//
// Created by amourao on 23-09-2019.
//

#include "../include/zdepth_raw_decoder.h"
ZDepthRawDecoder::ZDepthRawDecoder() {}

ZDepthRawDecoder::~ZDepthRawDecoder() {}

void ZDepthRawDecoder::Init(std::vector<unsigned char> parameter_data) {
  memcpy(&width_, &parameter_data[0], sizeof(int));
  memcpy(&height_, &parameter_data[4], sizeof(int));
}

cv::Mat ZDepthRawDecoder::Decode(FrameStruct& frame) {

  zdepth::DepthResult result =
      decompressor_.Decompress(frame.frame, width_, height_, decompressed_buffer_);
  if (result != zdepth::DepthResult::Success) {
    spdlog::error("Zdepth decompression error: {}", DepthResultString(result));
    // Return a blank texture on an error
    return cv::Mat::zeros(height_, width_, CV_16UC1);
  }

  return cv::Mat(height_, width_, CV_16UC1, decompressed_buffer_.data(),
                 cv::Mat::AUTO_STEP);
  ;
}

void* ZDepthRawDecoder::DecodeRaw(FrameStruct& frame) {
    zdepth::DepthResult result =
        decompressor_.Decompress(frame.frame, width_, height_, decompressed_buffer_);
    return decompressed_buffer_.data();
}
