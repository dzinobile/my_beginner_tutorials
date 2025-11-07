# Copyright 2025 Zinobile-Corp LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    text_color_arg = DeclareLaunchArgument(
        'listener_text_color',
        default_value='white',
        description='Set the text color for the listener'
    )

    text_color = LaunchConfiguration('listener_text_color')

    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='publisher',
        output='screen',
        arguments=['--ros-args', '--log-level', 'DEBUG']
    )
    
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='subscriber',
        output='screen',
        parameters=[{'text_color': text_color}]
    )

    return LaunchDescription([
        text_color_arg,
        talker_node,
        listener_node
    ])



