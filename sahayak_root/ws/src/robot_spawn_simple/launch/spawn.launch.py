#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots TurtleBot3 Burger driver."""

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('robot_spawn_simple')
    world = LaunchConfiguration('world')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'robot_spawn_simple'),
            ('executable', 'simple_driver'),
            ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='world.wbt',
            description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
        ),
        webots
    ])

#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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

"""Launch Webots and the controller."""

# import os
# import launch
# from webots_ros2_core.utils import ControllerLauncher
# from webots_ros2_core.webots_launcher import WebotsLauncher
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
#
#
# def generate_launch_description():
#     package_dir = get_package_share_directory('robot_spawn_simple')
#     world = LaunchConfiguration('world')
#     # Webots
#     webots = WebotsLauncher(
#         world=PathJoinSubstitution([package_dir, 'worlds', 'world.wbt'])
#     )
#
#     # Controller node
#     synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
#     controller = ControllerLauncher(
#         package='webots_ros2_examples',
#         executable='example_controller',
#         parameters=[{'synchronization': synchronization}],
#         output='screen'
#     )
#
#     return launch.LaunchDescription([
#         webots,
#         controller,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         ),
#     ])

#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
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
# """Launch Webots TurtleBot3 Burger driver."""
#
# import os
# import pathlib
# from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions.path_join_substitution import PathJoinSubstitution
# from launch import LaunchDescription
# from launch_ros.actions import Node
# import launch
# from ament_index_python.packages import get_package_share_directory
# from webots_ros2_core.webots_launcher import WebotsLauncher
#
#
# def generate_launch_description():
#     package_dir = get_package_share_directory('robot_spawn_simple')
#     world = LaunchConfiguration('world')
#     ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
#
#     webots = WebotsLauncher(
#         world=PathJoinSubstitution([package_dir, 'worlds', world])
#     )
#
#     diffdrive_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner.py',
#         output='screen',
#         prefix="bash -c 'sleep 10; $0 $@' ",
#         arguments=['diffdrive_controller'],
#     )
#
#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner.py',
#         output='screen',
#         prefix="bash -c 'sleep 10; $0 $@' ",
#         arguments=['joint_state_broadcaster'],
#     )
#
#     turtlebot_driver = Node(
#         package='webots_ros2_driver',
#         executable='driver',
#         output='screen',
#         remappings=[
#             ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')
#         ]
#     )
#
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{
#             'robot_description': '<robot name=""><link name=""/></robot>'
#         }],
#     )
#
#     footprint_publisher = Node(
#         package='tf2_ros',
#         executable='static_transform_publisher',
#         output='screen',
#         arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
#     )
#
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'world',
#             default_value='world.wbt',
#             description='Choose one of the world files from `/webots_ros2_turtlebot/world` directory'
#         ),
#         joint_state_broadcaster_spawner,
#         diffdrive_controller_spawner,
#         webots,
#         robot_state_publisher,
#         turtlebot_driver,
#         footprint_publisher,
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         )
#     ])