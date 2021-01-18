//
// Created by amourao on 12-09-2019.
//

#include "../include/libav_raw_decoder.h"

LibAvRawDecoder::LibAvRawDecoder() {}

LibAvRawDecoder::~LibAvRawDecoder() {}

void LibAvRawDecoder::Init(AVCodecParameters* codec_parameters) {
    av_register_all();

    codec_ = std::unique_ptr<AVCodec, AVCodecDeleter>(
        avcodec_find_decoder(codec_parameters->codec_id));
    codec_context_ = std::unique_ptr<AVCodecContext, AVCodecContextDeleter>(
        avcodec_alloc_context3(codec_.get()));
    if (!codec_context_) {
        spdlog::error("Failed to allocated memory for AVCodecContext.");
        exit(1);
    }

    if (avcodec_parameters_to_context(codec_context_.get(), codec_parameters) <
        0) {
        spdlog::error("Failed to copy codec params to codec context.");
        exit(1);
    }

    if (avcodec_open2(codec_context_.get(), codec_.get(), NULL) < 0) {
        spdlog::error("Failed to open codec through avcodec_open2.");
        exit(1);
    }
}

cv::Mat LibAvRawDecoder::Decode(FrameStruct& frame_struct) {
    AVPacketSharedP packet_av =
        std::shared_ptr<AVPacket>(av_packet_alloc(), AVPacketSharedDeleter);
    AVFrameSharedP frame_av =
        std::shared_ptr<AVFrame>(av_frame_alloc(), AVFrameSharedDeleter);

    packet_av->data = frame_struct.frame.data();
    packet_av->size = frame_struct.frame.size();

    cv::Mat img;
    int response = avcodec_send_packet(codec_context_.get(), packet_av.get());
    if (response >= 0) {
        // Return decoded output data (into a frame) from a decoder
        response = avcodec_receive_frame(codec_context_.get(), frame_av.get());
        if (response >= 0) {
            if (frame_struct.frame_type == 1 || frame_struct.frame_type == 2) {
                if (codec_context_->pix_fmt == AV_PIX_FMT_GRAY12LE ||
                    codec_context_->pix_fmt == AV_PIX_FMT_GRAY16BE) {
                    AVFrameToMatGray(frame_av, img);
                } else if (codec_context_->pix_fmt == AV_PIX_FMT_YUV420P) {
                    AVFrameToMatYUV(frame_av, img);
                    cv::cvtColor(img, img, CV_BGR2GRAY);
                    img.convertTo(img, CV_16UC1);
                    img *= (MAX_DEPTH_VALUE_12_BITS / MAX_DEPTH_VALUE_8_BITS);
                }
            } else {
                AVFrameToMatYUV(frame_av, img);
            }
        }
    }

    return img;
}

void* LibAvRawDecoder::DecodeRaw(FrameStruct& frame_struct) {
    AVPacketSharedP packet_av =
        std::shared_ptr<AVPacket>(av_packet_alloc(), AVPacketSharedDeleter);
    AVFrameSharedP frame_av =
        std::shared_ptr<AVFrame>(av_frame_alloc(), AVFrameSharedDeleter);

    packet_av->data = frame_struct.frame.data();
    packet_av->size = frame_struct.frame.size();

    cv::Mat img;
    int response = avcodec_send_packet(codec_context_.get(), packet_av.get());
    if (response >= 0) {
        response = avcodec_receive_frame(codec_context_.get(), frame_av.get());
        if (response >= 0) {
            int width = frame_av->width;
            int height = frame_av->height;

            // copy the pixel data into an RGBA buffer
            uint8_t* rgb_data[4];
            int rgb_linesize[4];

            SwsContext* conversion_ctx;
            conversion_ctx =
                sws_getContext(width, height, codec_context_->pix_fmt, width,
                               height, AV_PIX_FMT_RGBA, 0, 0, 0, 0);
            av_image_alloc(rgb_data, rgb_linesize, width, height,
                           AV_PIX_FMT_RGBA, 32);
            sws_scale(conversion_ctx, frame_av->data, frame_av->linesize, 0,
                      height, rgb_data, rgb_linesize);

            return rgb_data[0];
        }
    }
    return nullptr;
}