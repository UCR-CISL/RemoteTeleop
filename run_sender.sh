#!/bin/bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp LD_PRELOAD=/lib/x86_64-linux-gnu/libavutil.so.56 python3 v2/streaming/ros_sender.py --width 1280 --height 720 --fps 30