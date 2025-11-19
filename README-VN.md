# webrtc_rpi (C++ WebRTC for Raspberry Pi 4 - Debian 12)

C++ app dùng GStreamer `webrtcbin` để publish video từ camera gắn trực tiếp trên Raspberry Pi 4 (Debian 12 Bookworm) qua WebRTC. 

- Nguồn video:
  - Ưu tiên: `libcamerasrc` (stack mặc định trên Debian 12)
- Encoder:
  - Ưu tiên: VP8 (`vp8enc`)
- Signaling: WebSocket
- Vai trò WebRTC: Trình duyệt là Caller (gửi Offer), app trên Pi là Answerer (trả Answer). ICE hai chiều.

## Host Machine

**Clone repo:**

```
git clone https://github.com/phuoc-SET/webrtc-rpi-cpp.git
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

Sau khi build xong, copy file run webrtc_rpi trong thư mục build_arm64 ra ngoài folder chính (webrtc-rpi-cpp). 

**Copy sang Target Machine:**

```
scp -r [đường/dẫn/folder/webrtc-rpi-cpp]/ pi@[IP-RPI]:/home/pi/
```



## Target Machine (Debian 12 / Raspberry Pi 4)

**Lưu ý: trong khi cài  đặt môi trường cross-compile cho Host Machine, nên cài đặt thư viện cho Pi để tiết kiệm thời gian.**

```bash
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

Sau khi đã cross-compile và copy folder sang Pi:

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

Mở browser trên thiết bị kết nối cùng mạng với Raspberry:

```
http://[IP-RPI]:8080/
```



