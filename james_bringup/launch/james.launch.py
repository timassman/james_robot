# https://roboticsbackend.com/ros2-launch-file-example/
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
# https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#writing-launch-files
# https://www.youtube.com/watch?v=sl0exwcg3o8

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    create_bringup = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('create_bringup'),
                         'launch/create_2.launch')
        )
    )

    joy_teleop = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('create_bringup'),
                         'launch/joy_teleop.launch')
        )
    )

    kinect1_node = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        namespace="kinect",
    )

    ld.add_action(create_bringup)
    ld.add_action(joy_teleop)
    ld.add_action(kinect1_node)

    return ld

