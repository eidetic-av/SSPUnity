//
// Created by amourao on 26-06-2019.
//

#include <chrono>
#include <iostream>
#include <thread>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

#include <opencv2/imgproc.hpp>
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

#include "../include/network_subscriber.h"
#include "../include/libav_raw_decoder.h"
#include "../include/zdepth_raw_decoder.h"

NetworkSubscriber* subscriber;
LibAvRawDecoder* libav_decoder;
ZDepthRawDecoder* zdepth_decoder;

extern "C" {
__declspec(dllexport) void InitSubscriber(const char* host, int port, int poll_timeout_ms) {
    spdlog::set_level(spdlog::level::debug);
    av_log_set_level(AV_LOG_QUIET);

    srand(time(NULL) * getpid());

    spdlog::info("Initialising network subscriber with address: {}:{}", host,
                 port);

    subscriber = new NetworkSubscriber(host, port, poll_timeout_ms);
    subscriber->init();

    zdepth_decoder = new ZDepthRawDecoder();
    libav_decoder = new LibAvRawDecoder();

    spdlog::info(" ... success!");
}

__declspec(dllexport) void Close() {
    subscriber->~NetworkSubscriber();
    spdlog::info("Destroyed subscriber");
}

__declspec(dllexport) bool GetNextFramePtrs(void*& depth_frame_ptr,
                                            void*& color_frame_ptr) {
    if (!subscriber->HasNextFrame()) return false;

    subscriber->NextFrame();
    std::vector<FrameStruct> f_list = subscriber->GetCurrentFrame();

    for (FrameStruct f : f_list) {
        if (f.frame_type == 0) {
            libav_decoder->Init(getParams(f));
            color_frame_ptr = libav_decoder->DecodeRaw(f);
        } else if (f.frame_type == 1) {
            zdepth_decoder->Init(f.codec_data.data);
            depth_frame_ptr = zdepth_decoder->DecodeRaw(f);
        }
    }

    return true;
}
}