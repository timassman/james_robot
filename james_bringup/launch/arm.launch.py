# Starts the OpenManipulator-X arm: the ros2_control hardware interface with its
# controllers (joint_state_broadcaster, arm_controller, gripper_controller) and,
# optionally, MoveIt move_group for motion planning.
#
# RViz is not started here; run it on the monitoring laptop:
#   ros2 launch open_manipulator_x_moveit_config moveit_rviz.launch.py
#
# Usage:
#   ros2 launch james_bringup arm.launch.py                           # real arm + MoveIt
#   ros2 launch james_bringup arm.launch.py moveit:=false             # real arm only
#   ros2 launch james_bringup arm.launch.py use_fake_hardware:=true   # simulated arm, no hardware
#
# Warning: stopping this launch file disables the motor torque and the arm drops.
# Move the arm to the 'init' pose or support it by hand before pressing Ctrl+C.

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # run.sh maps the arm into the container as /dev/openmanipulator (udev symlink).
    # The ROBOTIS default is /dev/ttyUSB0, which does not exist in the container.
    ld.add_action(DeclareLaunchArgument(
        'port_name',
        default_value='/dev/openmanipulator',
        description='Serial port of the U2D2 Dynamixel adapter'))

    ld.add_action(DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Simulate the arm (no Dynamixels needed), e.g. for testing on the laptop'))

    ld.add_action(DeclareLaunchArgument(
        'moveit',
        default_value='true',
        description='Also start MoveIt move_group'))

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('open_manipulator_x_bringup'),
            'launch', 'hardware.launch.py')),
        launch_arguments={
            'port_name': LaunchConfiguration('port_name'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'start_rviz': 'false',
        }.items())

    # use_sim:=false is required on the real robot: the ROBOTIS move_group launch file
    # defaults to Gazebo simulation time (use_sim_time) and there is no /clock here.
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('open_manipulator_x_moveit_config'),
            'launch', 'move_group.launch.py')),
        launch_arguments={'use_sim': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('moveit')))

    ld.add_action(hardware)
    ld.add_action(move_group)

    return ld
