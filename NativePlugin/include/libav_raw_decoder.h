//
// Created by amourao on 12-09-2019.
//

#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <utils/logger.h>

#include <utils/libav_types.h>
#include <utils/video_utils.h>

#include <decoders/idecoder.h>

class LibAvRawDecoder : public IDecoder {
private:
  AVCodecSafeP codec_;
  AVCodecContextSafeP codec_context_;

public:
  LibAvRawDecoder();
  ~LibAvRawDecoder();
  void Init(AVCodecParameters *codec_parameters);
  cv::Mat Decode(FrameStruct &frame_struct);
  void *DecodeRaw(FrameStruct &frame_struct);
};
