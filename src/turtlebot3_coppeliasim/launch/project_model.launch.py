# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_coppeliasim'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    scene_dir = LaunchConfiguration(
            'scene_dir',
            default=os.path.join(
                get_package_share_directory('turtlebot3_coppeliasim'),
                'scenes',
                'turtlebot3_'+TURTLEBOT3_MODEL+'_ROS2.ttt'))

    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_coppeliasim'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_coppeliasim'),
        'urdf',
        urdf_file_name)

    return LaunchDescription([
        LogInfo(msg=['Execute Turtlebot3 CoppeliaSim!!']),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'),

        DeclareLaunchArgument(
            'scene_dir',
            default_value=scene_dir,
            description='Specifying scenes path'),

        #LogInfo(msg=[scene_dir]),
        
        ExecuteProcess(
            cmd=[['coppeliaSim.sh -f ',scene_dir,' -s 0']],
            shell=True
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rviz_dir, '/rviz2.launch.py'])),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),

        #uncomment for v3d
        # Node(
        #     package='turtlebot3_socket_bridge',
        #     executable='tcp_pub',
        #     name='socket_vel_publisher',
        #     output='screen'
        # ),
        Node(
            package='turtlebot3_main',
            executable='turtlebot3_main_node',
            name='turtlebot3_main',
            output='screen'
        ),
    ])
