# Shows the James description (crate + OpenManipulator-X) in RViz, to check the model
# against the real robot.
#
# Usage (laptop, james_description built with --symlink-install):
#   ros2 launch james_description display.launch.py                  # move the arm with sliders
#   ros2 launch james_description display.launch.py use_gui:=false   # follow the real arm
#
# Everything runs in the /preview namespace with the frame prefix 'preview/', so it can run
# next to arm.launch.py on the Jetson: the sliders publish /preview/joint_states (never
# /joint_states, which move_group uses) and the TF frames do not clash. With
# use_gui:=false the model follows the real /joint_states instead.
#
# Changes to the xacro files show up in RViz within a second (description_watcher), no
# restart needed. A new file in urdf/ needs a colcon build first.

import os
import tempfile

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

XACRO_ARGS = ('bottles', 'planks', 'held_bottle')


def launch_setup(context):
    share = get_package_share_directory('james_description')
    xacro_file = os.path.join(share, 'urdf', 'james.urdf.xacro')
    xacro_args = {name: LaunchConfiguration(name).perform(context) for name in XACRO_ARGS}
    description = xacro.process_file(xacro_file, mappings=xacro_args).toxml()
    use_gui = LaunchConfiguration('use_gui').perform(context).lower() == 'true'

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='preview',
            parameters=[{'robot_description': description, 'frame_prefix': 'preview/'}],
            remappings=[('joint_states', 'joint_states' if use_gui else '/joint_states')]),
        Node(
            package='james_description',
            executable='description_watcher',
            namespace='preview',
            parameters=[{
                'xacro_file': xacro_file,
                'xacro_args': ' '.join(f'{name}:={value}' for name, value in xacro_args.items()),
            }]),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'display.rviz')]),
    ]

    if use_gui:
        # joint_state_publisher_gui resets its sliders to zero on every new robot_description.
        # A fixed copy of the model keeps the arm pose when the crate model is reloaded.
        slider_model = os.path.join(tempfile.gettempdir(), 'james_description_sliders.urdf')
        with open(slider_model, 'w') as f:
            f.write(description)
        nodes.append(Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace='preview',
            arguments=[slider_model]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Move the arm with sliders; false: follow the real arm (/joint_states)'),
        DeclareLaunchArgument(
            'bottles',
            default_value='true',
            description='Show a bottle in every slot'),
        DeclareLaunchArgument(
            'planks',
            default_value='true',
            description='Show the wooden planks over column 1 and 4'),
        DeclareLaunchArgument(
            'held_bottle',
            default_value='false',
            description='Show a bottle in the gripper (top-down grasp)'),
        OpaqueFunction(function=launch_setup),
    ])
