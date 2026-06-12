#!/bin/bash

echo "Включаем стриминг камеры..."
gz topic -t /world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming \
  -m gz.msgs.Boolean -p "data: 1"

sleep 1

echo "Запуск просмотра видео с камеры дрона (UDP порт 5600)..."
gst-launch-1.0 -v udpsrc port=5600 \
  ! application/x-rtp, encoding-name=H264, payload=96 \
  ! rtpjitterbuffer \
  ! rtph264depay \
  ! avdec_h264 \
  ! videoconvert \
  ! ximagesink
