#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        
        # Start turtle_chase_node
        Node(
            package='turtle_pkg',
            executable='turtle_chase',
            name='turtle_chase',
            output='screen'
        ),
        
        # Start teleop key node (NEW TERMINAL)
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop_key',
            output='screen',
            prefix='xterm -e'
        )
    ]) 