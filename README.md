# webrtc_rpi (C++ WebRTC for Raspberry Pi 4 - Debian 12)

A C++ application using GStreamer's `webrtcbin` to publish video from a camera directly attached to a Raspberry Pi 4 (Debian 12 Bookworm) via WebRTC.

- **Video Source:**
  - Primary: `libcamerasrc` (default stack on Debian 12)
- **Encoder:**
  - Primary: VP8 (`vp8enc`)
- **Signaling:** WebSocket
- **WebRTC Role:** Browser acts as Caller (sends Offer), Pi app acts as Answerer (returns Answer). Bidirectional ICE.

## Host Machine (Cross-Compilation)

**Clone the repository:**

```
git clone [repository-url]
```

**Cross-compile**

```
cd webrtc-rpi-cpp/
sudo apt update
sudo apt install -y docker.io qemu-user-static
sudo usermod -aG docker $USER
newgrp docker
docker build -t webrtc-rpi-builder .
mkdir -p build_arm64
sudo docker run --rm -v $(pwd):/source -w /source/build_arm64 webrtc-rpi-builder /bin/bash -c "cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc)"
```

After a successful build, copy the `webrtc_rpi` executable from the `build_arm64` directory to the main project directory (`webrtc-rpi-cpp`).

**Copy to Target Machine:**

```
scp -r [/path/to/webrtc-rpi-cpp]/ pi@[RPI-IP]:/home/pi/
```



## Target Machine (Debian 12 / Raspberry Pi 4)

**Note: It is recommended to install the necessary libraries on the Pi while setting up the cross-compilation environment on the Host Machine to save time.** 

```
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
```

After cross-compiling and copying the project folder to the Pi:

```
cd webrtc-rpi-cpp/signaling
npm install
npm install ws
cd ../
```

**Run Test:**

```
chmod +x webrtc-rpi.sh
./webrtc-rpi.sh
```

Open a browser on a device connected to the same network as the Raspberry Pi and navigate to:

```
http://[RPI-IP]:8080/
```
# webrtc-rpi-cpp
