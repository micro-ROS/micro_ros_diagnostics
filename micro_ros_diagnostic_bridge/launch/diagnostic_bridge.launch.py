# Copyright (c) 2021 - for information on the respective copyright owner
# see the NOTICE file and/or the repository https://github.com/micro-ROS/system_modes.
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

import launch
import launch.actions
from launch.substitutions import LaunchConfiguration

import launch_ros.actions

logger = launch.substitutions.LaunchConfiguration('log_level')


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'lookup_table',
            description='Path to lookup table'),
        launch.actions.DeclareLaunchArgument(
            'log_level',
            default_value=['info'],
            description='Logging level'),
        launch.actions.DeclareLaunchArgument(
            'input_topic',
            default_value=['diagnostics_uros'],
            description='Remap for input topic'),
        launch.actions.DeclareLaunchArgument(
            'output_topic',
            default_value=['diagnostics'],
            description='Remap for output topic'),
        launch.actions.DeclareLaunchArgument(
            'namespace',
            default_value=[''],
            description='Namespace'),
        launch_ros.actions.Node(
            package='micro_ros_diagnostic_bridge',
            executable='diagnostic_bridge',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'lookup_table': LaunchConfiguration('lookup_table')}],
            remappings=[
                ('diagnostics_uros', LaunchConfiguration('input_topic')),
                ('diagnostics', LaunchConfiguration('output_topic'))
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', logger])
    ])
