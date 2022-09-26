#!/usr/bin/env python3
# Copyright 2022 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

import launch
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    pkg_jackal_diagnostics = get_package_share_directory('jackal_diagnostics')

    analyzer_params_filepath = PathJoinSubstitution(
        [pkg_jackal_diagnostics, 'config', 'diagnostics.yaml'])

    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath])
    diag_publisher = Node(
         package='jackal_diagnostics',
         executable='diagnostics_updater',
         output='screen')
    return launch.LaunchDescription([
        aggregator,
        diag_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=aggregator,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
