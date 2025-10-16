#!/usr/bin/env bash
set -e
echo "Installing system deps for Ubuntu 20.04 (Noetic)..."
sudo apt-get update

sudo apt-get install -y \
  build-essential \
  cmake \
  pkg-config \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav \
  gir1.2-gstreamer-1.0 \
  gir1.2-gst-plugins-base-1.0 \
  libgstreamer-plugins-bad1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer-allocators-1.0-dev \
  libopencv-dev \
  libjsoncpp-dev \
  nlohmann-json3-dev \
  libwebsocketpp-dev \
  libboost-system-dev \
  libboost-thread-dev \
  nodejs npm

# Python tools used elsewhere (optional)
sudo apt-get install -y python3-opencv python3-pip
pip3 install --user websockets

echo "Note: if gst webrtc plugin (webrtcbin) is missing, you may need to build gstreamer-plugins-bad with webrtc enabled."