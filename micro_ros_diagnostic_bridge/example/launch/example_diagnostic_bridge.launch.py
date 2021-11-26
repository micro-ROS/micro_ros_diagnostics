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

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

logger = LaunchConfiguration('log_level')


def generate_launch_description():

    bridge_launch = os.path.join(get_package_share_directory(
        'micro_ros_diagnostic_bridge'), 'launch/diagnostic_bridge.launch.py')

    lookup_table = os.path.join(get_package_share_directory(
        'micro_ros_diagnostic_bridge'), 'example_table.yaml')

    example_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bridge_launch),
        launch_arguments={
            'lookup_table': lookup_table
        }.items()
    )

    return LaunchDescription([
        example_bridge
    ])
