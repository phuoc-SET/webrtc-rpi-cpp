#!/usr/bin/env bash
set -euo pipefail

sudo apt update
sudo apt-get install -y \
  build-essential \
  cmake \
  pkg-config \
  gstreamer1.0-tools \
  gstreamer1.0-x \
  gstreamer1.0-alsa \
  gstreamer1.0-pulseaudio \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gstreamer1.0-nice \
  gstreamer1.0-libcamera \
  libnice10 \
  libnice-dev \
  gir1.2-gstreamer-1.0 \
  gir1.2-gst-plugins-base-1.0 \
  libopencv-dev \
  python3-opencv \
  libjsoncpp-dev \
  nlohmann-json3-dev \
  libwebsocketpp-dev \
  libboost-system-dev \
  libboost-thread-dev \
  nodejs npm

echo "Dependencies installed."