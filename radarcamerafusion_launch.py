#!/usr/bin/env python3

import os
import time
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MCity
      Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_to_camera_tf',
        # arguments = [x=-1 m, y=0 m, z=-1.5 m, yaw=pi/2 rad, pitch=0 rad, roll=pi/2 rad, frame_id=radar, child_frame_id=front_camera]
        arguments=['-1', '0', '-1.5', '1.57', '0', '1.405', 'radar', 'front_camera']
      ),
      Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc',
        namespace='front_camera',
        remappings=[('/front_camera/image','/front_camera/image_raw')]
      ),
      Node(
        package='rviz2',
        executable='rviz2',
        namespace='',
        name='rviz',
        arguments=['-d', '/home/junnanshimizu/.rviz2/radarcamerafusion.rviz']
      ),
      Node(
        package='radarcamera_fusion',
        executable='radarcamera_node'
      )
   ])
