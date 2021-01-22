//
// Created by amourao on 27-09-2019.
//

#pragma once

#include <zmq.hpp>
#undef max
// https://stackoverflow.com/questions/27442885/syntax-error-with-stdnumeric-limitsmax
#include <zmq_addon.hpp>

#include <readers/ireader.h>
#include <structs/frame_struct.hpp>

#include <utils/video_utils.h>

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
  NetworkSubscriber(std::string host, int port, int poll_timeout_ms);
  void init();

  ~NetworkSubscriber();

  bool HasNextFrame();

  void NextFrame();

  std::vector<FrameStruct> GetCurrentFrame();

  unsigned int GetCurrentFrameId();
};
