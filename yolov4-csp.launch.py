from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  darknet_ros_share_dir = get_package_share_directory('darknet_ros')
  network_param_file = darknet_ros_share_dir + '/config/yolov4-csp.yaml'

  darknet_ros_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([darknet_ros_share_dir + '/launch/darknet_ros.launch.py']),
      launch_arguments={'network_param_file': network_param_file}.items()
  )

  camera = Node(
    package="v4l2_camera",
    executable="v4l2_camera_node",
    parameters=[
      {'video_device'     : "/dev/video0"},
    ])
    
  rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        namespace='',
        name='rviz',
        arguments=['-d', '/home/junnanshimizu/.rviz2/radarcamerafusion.rviz']
      )
  
  image_proc = Node(
        package='image_proc',
        executable='image_proc',
        name='image_proc',
        namespace='front_camera',
        remappings=[('/front_camera/image',"/front_camera/image_raw")]
      )
  
  tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_to_camera_tf',
        # arguments = [x=-1 m, y=0 m, z=-1.5 m, yaw=pi/2 rad, pitch=0 rad, roll=pi/2 rad, frame_id=radar, child_frame_id=front_camera]
        arguments=['-1', '0', '-1.5', '1.57', '0', '1.39', 'radar', 'front_camera']
      )
      
  radarcamera_node = Node(
    package='radarcamera_fusion',
    executable='radarcamera_node'
  )

  return LaunchDescription([
    rviz2,
    darknet_ros_launch,
    image_proc,
    tf2,
    radarcamera_node,
    # if you want to disable camera node, remove the following line.
    # camera,
  ])
