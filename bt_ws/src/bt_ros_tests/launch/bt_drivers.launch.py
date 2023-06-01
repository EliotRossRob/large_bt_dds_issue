#! /usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments

  adder = Node(
        package='bt_ros_tests',
        executable='adding_server',
    )

  adder_pt2 = Node(
        package='bt_ros_tests',
        executable='adding_server_pt2',
    )

  sleep = Node(
        package='bt_ros_tests',
        executable='sleep_server',
    )

  speaking = Node(
        package='bt_ros_tests',
        executable='speaking_server',
    )

  listener = Node(
        package='bt_ros_tests',
        executable='listening_server'
  )

  nodes = [
        adder,
        sleep,
        speaking,
        listener
    ]

  return LaunchDescription(nodes)