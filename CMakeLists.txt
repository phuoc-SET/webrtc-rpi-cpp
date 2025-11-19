cmake_minimum_required(VERSION 3.15)
project(webrtc_rpi CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(nlohmann_json REQUIRED)

# GStreamer
pkg_check_modules(GST REQUIRED
  gstreamer-1.0>=1.20
  gstreamer-sdp-1.0
  gstreamer-webrtc-1.0
  gstreamer-video-1.0
)
pkg_check_modules(GST_BASE REQUIRED gstreamer-plugins-base-1.0)
pkg_check_modules(GST_BAD REQUIRED gstreamer-plugins-bad-1.0)

add_executable(webrtc_rpi
  src/webrtc_rpi.cpp
)

target_include_directories(webrtc_rpi PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GST_BASE_INCLUDE_DIRS}
  ${GST_BAD_INCLUDE_DIRS}
)

target_link_directories(webrtc_rpi PRIVATE
  ${GST_LIBRARY_DIRS}
  ${GST_BASE_LIBRARY_DIRS}
  ${GST_BAD_LIBRARY_DIRS}
)

target_link_libraries(webrtc_rpi PRIVATE
  ${GST_LIBRARIES}
  ${GST_BASE_LIBRARIES}
  ${GST_BAD_LIBRARIES}
  Boost::system
  Boost::thread
  Threads::Threads
  nlohmann_json::nlohmann_json
)

# Unstable WebRTC API
target_compile_definitions(webrtc_rpi PRIVATE
  GST_USE_UNSTABLE_API
)

# Pass pkg-config CFLAGS as options (avoid turning into -D macros)
target_compile_options(webrtc_rpi PRIVATE
  ${GST_CFLAGS_OTHER}
  ${GST_BASE_CFLAGS_OTHER}
  ${GST_BAD_CFLAGS_OTHER}
)

if (CMAKE_BUILD_TYPE MATCHES "Release")
  target_compile_options(webrtc_rpi PRIVATE -O3)
endif()