import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from resolve_robotics_uri_py import resolve_robotics_uri_py

def generate_launch_description():

    # Load URDF file
    robot_description = None
    urdf_path = os.path.abspath(resolve_robotics_uri_py.resolve_robotics_uri("package://R1Mk3/robots/R1Mk3Gazebo/model.urdf"))

    with open(urdf_path, 'r') as f:
        robot_description = f.read()
    if not robot_description:
        print('Cannot load robot description.')
        exit(1)

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default = 'false')
    arguments = DeclareLaunchArgument\
    (
        'use_sim_time',
        default_value = 'false',
        description = 'Use simulation clock from Gazebo if true.'
    )

    # Declare robot state publisher node
    node = Node\
    (
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        arguments = ['--ros-args', '--log-level', 'info'],
        # remappings = [('joint_states', 'joint_states_full')]
    )

    # Declare launch description
    launch_description = LaunchDescription([arguments, node])

    return launch_description