#!/bin/bash

ros2 launch realsense2_camera rs_get_params_from_yaml_launch.py camera_name:=D435 camera_namespace:=robot config_file:="/root/ros2_ws/src/realsense-ros/realsense2_camera/config/config.yaml"
