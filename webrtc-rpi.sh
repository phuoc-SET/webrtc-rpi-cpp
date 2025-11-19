#!/bin/bash

# Kill process - Ctrl+C
trap "kill 0" EXIT

# Show IP RPI
IP_ADDRESS=$(hostname -I | awk '{print $1}')
echo "=== RASPBERRY PI IP: $IP_ADDRESS ==="

# Run Signaling Server in background
echo ">>> Starting Signaling Server..."
cd signaling 
PORT=8080 node server.js > ../signaling.log 2>&1 &
SERVER_PID=$!
cd ..
sleep 2


echo ">>> Starting WebRTC App..."
./webrtc_rpi --signaling-url ws://127.0.0.1:8080 --width 320 --height 240 --fps 30 --encoder vp8 --source libcamera --role answer

wait
