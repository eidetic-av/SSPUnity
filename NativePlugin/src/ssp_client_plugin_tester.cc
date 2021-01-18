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

// External symbols
typedef void(__stdcall *f_init_subscriber)(const char *host, int port, int poll_timeout_ms);
typedef void(__stdcall *f_close_subscriber)();
typedef bool(__stdcall *f_get_next_frame_ptrs)(void *&depth_frame_ptr,
                                               void *&color_frame_ptr);

int main(int argc, char *argv[]) {
    spdlog::set_level(spdlog::level::debug);
    av_log_set_level(AV_LOG_QUIET);

    srand(time(NULL) * getpid());

    HINSTANCE hGetProcIDDLL = LoadLibrary("ssp_client_plugin.dll");
    if (!hGetProcIDDLL) {
        spdlog::error("Failed to load Plugin dll.");
        return EXIT_FAILURE;
    }
    spdlog::info("Successfully loaded plugin dll");

    // load external functions from dll
    f_init_subscriber init_subscriber =
        (f_init_subscriber)GetProcAddress(hGetProcIDDLL, "InitSubscriber");
    f_close_subscriber close_subscriber =
        (f_close_subscriber)GetProcAddress(hGetProcIDDLL, "Close");
    f_get_next_frame_ptrs get_next_frame_ptrs =
        (f_get_next_frame_ptrs)GetProcAddress(hGetProcIDDLL,
                                              "GetNextFramePtrs");

    init_subscriber("localhost", 9999, 5);

    void *depth_frame_data;
    void *color_frame_data;

    while (1) {
      auto new_frame = get_next_frame_ptrs(depth_frame_data, color_frame_data);
      spdlog::debug("new frame? {} ", new_frame);
    }

    close_subscriber();

    return 0;
}
