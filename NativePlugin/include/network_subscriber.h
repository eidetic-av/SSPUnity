//
// Created by amourao on 27-09-2019.
//

#pragma once

#include <zmq.hpp>
#undef max
// https://stackoverflow.com/questions/27442885/syntax-error-with-stdnumeric-limitsmax
#include <zmq_addon.hpp>

#include <structs/frame_struct.hpp>
#include <readers/ireader.h>

#include "/Sensor-Stream-Pipe/utils/video_utils.h""

#define POLL_TIMEOUT_MS 500

class NetworkSubscriber {

private:
  uint64_t last_time_;
  uint64_t start_time_;
  uint64_t rec_frames_;
  double rec_mbytes_;

  int current_frame_counter_;

  std::unordered_map<std::string, double> rec_mbytes_per_stream_;
  std::vector<FrameStruct> current_frame_internal_;

  std::string host_;
  int port_;
  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  zmq::poller_t<> poller_;

public:
  NetworkSubscriber(std::string host, int port);
  void init();

  ~NetworkSubscriber();

  bool HasNextFrame();

  void NextFrame();

  std::vector<FrameStruct> GetCurrentFrame();

  unsigned int GetCurrentFrameId();

};