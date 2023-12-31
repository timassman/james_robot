# Source: https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/kinect_xbox_360.launch.py

# Requirements:
#   A kinect2
#   Install kinect2_ros2 package (use this fork: https://github.com/timassman/kinect2_ros2.git)

# About launch file arguments:
# https://answers.ros.org/question/382000/ros2-makes-launch-files-crazy-too-soon-to-be-migrating/

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':True,
          'qos':1}]

    remappings=[
          ('rgb/image', '/kinect2/rgb/image_raw'),
          ('rgb/camera_info', '/kinect2/rgb/camera_info'),
          ('depth/image', '/kinect2/depth_registered/image_raw')]

    on_robot = LaunchConfiguration('on_robot')

    return LaunchDescription([

        DeclareLaunchArgument(
            'on_robot',
            default_value='true',
            description='On the headless robot, run kinect2, rtabmap and no rviz'
        ),

        # Nodes to launch
        Node(
            package='kinect2_ros2', executable='kinect2_ros2_node', output='screen',
            parameters=[{'depth_registration':True}],
            namespace="kinect2",
            condition=IfCondition(on_robot)),

        # Optical rotation
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "kinect2_rgb"],
            condition=IfCondition(on_robot)),

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap",
            condition=IfCondition(on_robot)),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
            namespace="rtabmap",
            condition=IfCondition(on_robot)),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap",
            condition=IfCondition(on_robot)),
    ])