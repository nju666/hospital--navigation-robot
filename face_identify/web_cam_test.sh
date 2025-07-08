#!/bin/bash

echo "Killing previous python and ros2 processes..."
pkill -f video_stream.py
pkill -f my_ws_server.py
pkill -f originbot_face_identify_node
pkill -f face_identify_launch.py  # 建议加上，防止之前launch残留进程

sleep 1

echo "Starting video stream..."
nohup python3 /userdata/dev_ws/src/originbot/originbot_face_identify/video_stream.py > video_stream.log 2>&1 &

sleep 2

echo "Starting face identify node..."
nohup ros2 launch originbot_face_identify face_identify_launch.py > face_identify.log 2>&1 &

sleep 2

echo "Starting websocket server..."
nohup python3 /userdata/dev_ws/src/originbot/originbot_face_identify/my_ws_server.py > websocket_server.log 2>&1 &

echo "All services started!"
echo "Check video_stream.log, face_identify.log, websocket_server.log for logs."
