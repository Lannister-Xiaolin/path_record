#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
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
import os
from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Parameters
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    declare_record_with_start = DeclareLaunchArgument(
        'record_with_start',
        default_value='true',
        description='record_with_start')
    # Nodes launching commands

    config = os.path.join(
        get_package_share_directory('path_record'),
        'params',
        'path_record.yaml'
    )

    start_path_record_server_node_cmd = launch_ros.actions.Node(
        package='path_record',
        executable='path_record_server',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time"),
                     "record_with_start": LaunchConfiguration("record_with_start")}, config])

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_record_with_start)
    ld.add_action(start_path_record_server_node_cmd)
    return ld
