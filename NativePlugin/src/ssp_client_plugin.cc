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

#include "../utils/logger.h"

#include "../readers/network_reader.h"
#include "../utils/video_utils.h"
#include "../utils/image_converter.h"

NetworkReader* reader;
ZDepthDecoder* decoder;

extern "C" {
    __declspec (dllexport) void InitNetworkReader(int port)
    {
        spdlog::set_level(spdlog::level::debug);
        av_log_set_level(AV_LOG_QUIET);

        srand(time(NULL) * getpid());

        reader = new NetworkReader(port);
        reader->init();

        decoder = new ZDepthDecoder();
    }

    __declspec (dllexport) void Close()
    {
        reader->~NetworkReader();
    }

    __declspec (dllexport) bool ReaderHasNextFrame() {
        return reader->HasNextFrame();
    }

    // TODO change this to set two void pointers as reference instead
    // one for depth and another for colour
    __declspec (dllexport) void* GetNextFramePtr(u_long &frameAge)
    {
        reader->NextFrame();
        FrameStruct f = reader->GetCurrentFrame()[0];
        frameAge = f.timestamps.back() - f.timestamps.at(1);;
        return decoder->DecodeRaw(f);
    }
}