#include <chrono>
#include <iostream>
#include <thread>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include <zmq.hpp>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include <utils/image_converter.h>
#include <utils/logger.h>
#include <utils/video_utils.h>

#include "../include/libav_raw_decoder.h"
#include "../include/network_subscriber.h"
#include "../include/zdepth_raw_decoder.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

std::vector<NetworkSubscriber *> subscribers;
std::vector<LibAvRawDecoder *> libav_decoders;
std::vector<ZDepthRawDecoder *> zdepth_decoders;

extern "C" {

struct Skeleton {
  float hip_x;
  float hip_y;
  float hip_z;
};

EXPORT void InitSubscriber(const char *host, int port, int poll_timeout_ms,
                           int &subscriber_idx) {

  spdlog::set_level(spdlog::level::info);

    av_log_set_level(AV_LOG_QUIET);

    srand(time(NULL) * getpid());

    spdlog::info("Initialising network subscriber with address: {}:{}", host,
                 port);

  auto subscriber = new NetworkSubscriber(host, port, poll_timeout_ms);
    subscriber->init();

  subscriber_idx = subscribers.size();
  subscribers.push_back(subscriber);

  zdepth_decoders.push_back(new ZDepthRawDecoder());
  libav_decoders.push_back(new LibAvRawDecoder());

    spdlog::info(" ... success!");
}

EXPORT void Close(int subscriber_idx) {
  auto subscriber = subscribers[subscriber_idx];
    subscriber->~NetworkSubscriber();
  subscribers.erase(subscribers.begin() + subscriber_idx);
  libav_decoders.erase(libav_decoders.begin() + subscriber_idx);
  zdepth_decoders.erase(zdepth_decoders.begin() + subscriber_idx);
    spdlog::info("Destroyed subscriber");
}

EXPORT bool GetNextFramePtrs(int subscriber_idx, void *&depth_frame_ptr,
                             void *&color_frame_ptr, void *&joint_data_ptr) {
  auto subscriber = subscribers[subscriber_idx];
  if (!subscriber->HasNextFrame())
    return false;

    subscriber->NextFrame();
    std::vector<FrameStruct> f_list = subscriber->GetCurrentFrame();

    for (FrameStruct f : f_list) {
    if (f.frame_data_type == 5) {
    } else if (f.frame_type == 0) {
      auto libav_decoder = libav_decoders[subscriber_idx];
            libav_decoder->Init(getParams(f));
      av_free(color_frame_ptr);
            color_frame_ptr = libav_decoder->DecodeRaw(f);
        } else if (f.frame_type == 1) {
      auto zdepth_decoder = zdepth_decoders[subscriber_idx];
            zdepth_decoder->Init(f.codec_data.data);
            depth_frame_ptr = zdepth_decoder->DecodeRaw(f);
        }
    }

    return true;
}
}