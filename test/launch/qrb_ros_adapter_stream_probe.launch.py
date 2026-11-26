# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# The ROS side of the end-to-end test: the ROS IPC Bridge, with
# RosStreamProbeNode loaded into its container. Arguments left empty take the
# probe's defaults (ros2 run qrb_ros_adapter_stream_probe ros_stream_probe_node --help).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

ARGUMENTS = {
    'adapter_config': "the Adapter's static config, given to the HCM side too",
    'fps': 'frames per second a sender sends, 0 for no pause',
    'payload_bytes': "a sender's payload",
    'duration': 'seconds to run, 0 until stopped',
    'warmup': 'seconds a receiver does not measure',
    'log': 'text log file',
}
# A launch argument is a string; the probe takes these as numbers only.
NUMBERS = {'fps', 'payload_bytes', 'duration', 'warmup'}


def typed(name, value):
    if name not in NUMBERS:
        return value
    for number in (int, float):
        try:
            return number(value)
        except ValueError:
            pass
    return value  # the probe names the argument that is not a number


def load_probe(context):
    parameters = {}
    for name in ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value:
            parameters[name] = typed(name, value)
    return [
        LoadComposableNodes(
            target_container='/qrb_ros_adapter_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='qrb_ros_adapter_stream_probe',
                    plugin='qrb::adapter::stream_probe::RosStreamProbeNode',
                    parameters=[parameters],
                )
            ],
        )
    ]


def generate_launch_description():
    bridge = os.path.join(
        get_package_share_directory('qrb_ros_adapter'), 'launch', 'ros_ipc_bridge.launch.py')
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value='', description=text)
         for name, text in ARGUMENTS.items()] + [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge)),
            OpaqueFunction(function=load_probe),
        ])
