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

#include <utils/logger.h>

#include <utils/video_utils.h>
#include <utils/image_converter.h>

#include "../include/network_subscriber.h"

int main(int argc, char *argv[]) {

  spdlog::set_level(spdlog::level::debug);
  av_log_set_level(AV_LOG_QUIET);

  srand(time(NULL) * getpid());

  try {

    if (argc < 3) {
      std::cerr << "Usage: ssp_client_subscriber <host> <port>"
                << std::endl;
      return 1;
    }
    std::string log_level = "debug";
    std::string log_file = "";

    std::string host = argv[1];
    int port = std::stoi(argv[2]);
    NetworkSubscriber subscriber(host, port);

    subscriber.init();

    std::unordered_map<std::string, std::shared_ptr<IDecoder>> decoders;

    bool imgChanged = false;
    while (subscriber.HasNextFrame()) {
      subscriber.NextFrame();
      std::vector<FrameStruct> f_list = subscriber.GetCurrentFrame();

      for (FrameStruct f : f_list) {
        std::string decoder_id = f.stream_id + std::to_string(f.sensor_id);

        cv::Mat img;

        imgChanged = FrameStructToMat(f, img, decoders);

        if (imgChanged && !img.empty()) {

          // Manipulate image to show as a color map
          if (f.frame_type == 1) {
            if (img.type() == CV_16U) {
              // Compress images to show up on a 255 valued color map
              img *= (MAX_DEPTH_VALUE_8_BITS / (float)MAX_DEPTH_VALUE_12_BITS);
            }
            cv::Mat imgOut;

            img.convertTo(imgOut, CV_8U);
            cv::applyColorMap(imgOut, img, cv::COLORMAP_JET);
          } else if (f.frame_type == 2) {

            double max = 1024;
            img *= (MAX_DEPTH_VALUE_8_BITS / (float)max);
            img.convertTo(img, CV_8U);
          }

          cv::namedWindow(decoder_id);
          cv::imshow(decoder_id, img);
          cv::waitKey(1);
        }
      }
    }

  } catch (std::exception &e) {
    spdlog::error(e.what());
  }

  return 0;
}
