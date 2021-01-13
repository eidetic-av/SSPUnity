cmake_minimum_required(VERSION 3.16)
project(SSPUnity)

if (DEFINED ENV{SSP_DIR})
	message(STATUS "Using SensorStreamPipe source at $ENV{SSP_DIR}")
else()
	message(FATAL_ERROR "You must set the SSP_DIR environment variable \
		to a valid Sensor-Stream-Pipe source directory.")
endif()

set(default_build_type "Release")

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED PATHS "/libs/lib/opencv")
find_package(FFmpeg REQUIRED PATHS "/libs/lib/ffmpeg")

include_directories("/libs/include")
include_directories($ENV{SSP_DIR})
link_directories("/libs/lib")

set(LIB_FILES zmq zdepth zstd yaml-cpp swscale avcodec avformat swresample avutil avfilter)

set(SOURCE_FILES $ENV{SSP_DIR}/readers/image_reader.cc $ENV{SSP_DIR}/readers/multi_image_reader.cc $ENV{SSP_DIR}/structs/frame_struct.hpp $ENV{SSP_DIR}/utils/utils.cc $ENV{SSP_DIR}/utils/video_utils.cc $ENV{SSP_DIR}/utils/image_decoder.cc $ENV{SSP_DIR}/utils/image_converter.cc)
set(DECODER_SOURCE_FILES $ENV{SSP_DIR}/decoders/libav_decoder.cc $ENV{SSP_DIR}/decoders/zdepth_decoder.cc)

add_library(ssp_client_plugin SHARED src/ssp_client_plugin.cc $ENV{SSP_DIR}/readers/network_reader.cc ${DECODER_SOURCE_FILES} ${SOURCE_FILES})

target_link_libraries(ssp_client_plugin ${LIB_FILES})
target_link_libraries(ssp_client_plugin ${OpenCV_LIBS})
