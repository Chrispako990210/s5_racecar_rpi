#!/usr/bin/env bash

rosbag record -O equipe4.bag/tf /tf_static /racecar/map /racecar/status /racecar/scan /racecar/image_detections /racecar/raspicam_node/image/compressed
