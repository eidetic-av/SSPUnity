cmake_minimum_required(VERSION 3.16)
project(SSPUnity)

if (DEFINED ${SSP_DIR})
	message(STATUS "Using SensorStreamPipe source at ${SSP_DIR}")
elseif (DEFINED ENV{SSP_DIR})
	message(STATUS "Using SensorStreamPipe source at $ENV{SSP_DIR}")
	set(SSP_DIR $ENV{SSP_DIR})
else()
	message(FATAL_ERROR "You must set the SSP_DIR variable \
	to a valid Sensor-Stream-Pipe source directory.")
endif()

include_directories(/libs/include)
include_directories(${SSP_DIR})
include_directories(include)

find_package(Threads REQUIRED)

if(ANDROID)
	set(LIB_PATH /libs/android/${ANDROID_ABI})
	find_package(OpenCV 3.4.1 REQUIRED java)
else()
	set(LIB_PATH /libs/lib/)
	set(OpenCV_DIR ${LIB_PATH}/opencv)
	find_package(OpenCV 3.4.1 REQUIRED)
	find_package(FFmpeg REQUIRED PATHS ${LIB_PATH}/ffmpeg)
endif()

link_directories(${LIB_PATH})

set(LIB_FILES zmq zdepth zstd yaml-cpp swscale avcodec avformat swresample avutil avfilter)
if(ANDROID)
	set(LIB_FILES ${LIB_FILES} opencv_java)
else()
	set(LIB_FILES ${LIB_FILES} k4a k4abt k4arecord Kinect20)
endif()

set(SOURCE_FILES ${SSP_DIR}/structs/frame_struct.hpp ${SSP_DIR}/utils/utils.cc 
	${SSP_DIR}/utils/video_utils.cc ${SSP_DIR}/utils/image_decoder.cc
	${SSP_DIR}/utils/image_converter.cc ${SSP_DIR}/readers/network_reader.cc
	${SSP_DIR}/decoders/libav_decoder.cc ${SSP_DIR}/decoders/zdepth_decoder.cc 
	src/libav_raw_decoder.cc src/zdepth_raw_decoder.cc)

set(SERVER_SOURCE_FILES src/k4a_reader.cc src/kinect2_reader.cc
	${SSP_DIR}/encoders/libav_encoder.cc ${SSP_DIR}/encoders/zdepth_encoder.cc
	${SSP_DIR}/encoders/null_encoder.cc ${SSP_DIR}/readers/video_file_reader.cc
	${SSP_DIR}/utils/kinect_utils.cc)

# only add executables for the server and test clients to desktop platforms
if (NOT(ANDROID))
	add_executable(ssp_server_publish src/ssp_server_publish.cc ${SOURCE_FILES}
	       	${SERVER_SOURCE_FILES})
	target_link_libraries(ssp_server_publish ${LIB_FILES} ${OpenCV_LIBS})

	add_executable(ssp_client_subscriber src/network_subscriber.cc
	       	src/ssp_client_subscriber.cc ${SOURCE_FILES})
	target_link_libraries(ssp_client_subscriber ${LIB_FILES} ${OpenCV_LIBS})
	
	add_executable(ssp_client_plugin_tester src/ssp_client_plugin_tester.cc
	       	src/network_subscriber.cc ${SOURCE_FILES})
	target_link_libraries(ssp_client_plugin_tester ${LIB_FILES} ${OpenCV_LIBS})
endif()

add_library(ssp_client_plugin SHARED src/ssp_client_plugin.cc src/network_subscriber.cc ${SOURCE_FILES})
target_link_libraries(ssp_client_plugin ${LIB_FILES} ${OpenCV_LIBS})

# move compiled library into Unity package
if(WIN32)
	set(UNITY_LIB_DIR ${SSPUnity_SOURCE_DIR}/../UnityPackage/Plugins/Win64/)
elseif(ANDROID)
	set(UNITY_LIB_DIR ${SSPUnity_SOURCE_DIR}/../UnityPackage/Plugins/Android/lib/${ANDROID_ABI}/)
endif()

add_custom_command(TARGET ssp_client_plugin POST_BUILD COMMAND
	${CMAKE_COMMAND} -E copy $<TARGET_FILE:ssp_client_plugin> 
	${UNITY_LIB_DIR})

# move config for server into build directory
if(NOT(ANDROID))
	add_custom_command(TARGET ssp_server_publish POST_BUILD COMMAND
		${CMAKE_COMMAND} -E copy ${SSPUnity_SOURCE_DIR}/extra/default_server_config.yaml
		$<TARGET_FILE_DIR:ssp_server_publish>/config.yaml)
endif()

# move required dynamic libs for server into build directory
if (WIN32)
	add_custom_command(TARGET ssp_server_publish POST_BUILD 
		COMMAND ${CMAKE_COMMAND} -E copy_directory
		${SSPUnity_SOURCE_DIR}/extra/win $<TARGET_FILE_DIR:ssp_server_publish>)
endif()
