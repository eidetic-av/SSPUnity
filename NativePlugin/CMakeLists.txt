cmake_minimum_required(VERSION 3.16)
project(SSPUnity)

if (DEFINED ENV{SSP_DIR})
	message(STATUS "Using SensorStreamPipe source at $ENV{SSP_DIR}")
else()
	message(FATAL_ERROR "You must set the SSP_DIR environment variable \
		to a valid Sensor-Stream-Pipe source directory.")
endif()

set(CMAKE_BUILD_TYPE "Release")

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED PATHS "/libs/lib/opencv")
find_package(FFmpeg REQUIRED PATHS "/libs/lib/ffmpeg")

include_directories("/libs/include")
include_directories($ENV{SSP_DIR})
include_directories("include")

link_directories("/libs/lib")

set(LIB_FILES zmq zdepth zstd yaml-cpp swscale avcodec avformat swresample avutil avfilter
	k4a k4abt k4arecord)

set(SOURCE_FILES $ENV{SSP_DIR}/readers/image_reader.cc $ENV{SSP_DIR}/readers/multi_image_reader.cc 
	$ENV{SSP_DIR}/structs/frame_struct.hpp $ENV{SSP_DIR}/utils/utils.cc $ENV{SSP_DIR}/utils/video_utils.cc 
	$ENV{SSP_DIR}/utils/image_decoder.cc $ENV{SSP_DIR}/utils/image_converter.cc 
	$ENV{SSP_DIR}/readers/network_reader.cc $ENV{SSP_DIR}/decoders/libav_decoder.cc 
	$ENV{SSP_DIR}/decoders/zdepth_decoder.cc)

set(SERVER_EXTRA_SOURCE_FILES $ENV{SSP_DIR}/encoders/libav_encoder.cc $ENV{SSP_DIR}/encoders/zdepth_encoder.cc $ENV{SSP_DIR}/encoders/null_encoder.cc $ENV{SSP_DIR}/readers/video_file_reader.cc $ENV{SSP_DIR}/readers/image_reader.cc $ENV{SSP_DIR}/readers/kinect_reader.cc $ENV{SSP_DIR}/readers/multi_image_reader.cc $ENV{SSP_DIR}/utils/kinect_utils.cc)

add_executable(ssp_server_publish src/ssp_server_publish.cc ${SOURCE_FILES} ${SERVER_EXTRA_SOURCE_FILES})
target_link_libraries(ssp_server_publish ${LIB_FILES} ${OpenCV_LIBS})

add_executable(ssp_client_subscriber src/network_subscriber.cc src/ssp_client_subscriber.cc ${SOURCE_FILES})
target_link_libraries(ssp_client_subscriber ${LIB_FILES} ${OpenCV_LIBS})

add_library(ssp_client_plugin SHARED src/ssp_client_plugin.cc ${SOURCE_FILES})
target_link_libraries(ssp_client_plugin ${LIB_FILES} ${OpenCV_LIBS})

set(UNITY_PACKAGE_DIR "../../UnityPackage/Plugins/Win64/")
add_custom_command(TARGET ssp_client_plugin POST_BUILD COMMAND
       	${CMAKE_COMMAND} -E copy $<TARGET_FILE:ssp_client_plugin> ${UNITY_PACKAGE_DIR})
