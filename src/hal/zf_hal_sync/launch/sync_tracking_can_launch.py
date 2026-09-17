# Copyright 2022 Clyde McQueen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example pipeline using rclcpp_components.

This launches the gscam and other nodes into a container so that they run in the same process.
"""
import launch
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import ComposableNodeContainer 
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

def launch_setup(context, *args, **kwargs):
    # TODO: run this cmd in a single process
    # trigger_cmd = 'tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 30 1000'
    # os.system('echo %s|sudo -S %s' % ('nvidia', trigger_cmd)) 
    
    can_interfaces = LaunchConfiguration('can_interfaces')
    sync_ns = "sync_ns"


    can_names = str(can_interfaces.perform(context))
    can_name_list = can_names.split()

    radar_node = ComposableNode(
        package="zf_hal_sync",
        plugin="zf::driver::synchronization::HalRadarPackNode",
        name="hal_radar_node",
        namespace=sync_ns,
        parameters=[
            {
                # "can_interface": can_name,
                'enable_debug': LaunchConfiguration('enable_debug'),
            }
        ]
        # Future-proof: enable zero-copy IPC when it is available
        # https://github.com/ros-perception/image_common/issues/212
        # extra_arguments=[{"use_intra_process_comms": True}],
    )

    sync_node = ComposableNode(
        package="zf_hal_sync",
        plugin="zf::driver::synchronization::SyncTrackingCanNode",
        name="hal_sync_tracking_can_node",
        namespace=sync_ns,
        parameters=[
            {
                "canInterfaces": can_name_list,
                'enable_debug': LaunchConfiguration('enable_debug'),
            }
        ]
    )

    return [
            # trigger_cmd,

            ComposableNodeContainer(
                name="sync_container",
                namespace=sync_ns,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    
                    radar_node,
                    sync_node,
                ],
                output="screen",
                # emulate_tty = True,
            )
        ]

def generate_launch_description():

    # # Get the path to the user's home directory
    # home_dir = os.path.expanduser("~")
    # # Define the path to the file you want to save
    # file_path = os.path.join(home_dir, "Pictures/camera/")

    launch_arg_can_interface = DeclareLaunchArgument(
        'can_interfaces',
        # default_value='can1 can0 pcan0' # order 5G4T, vehicle, and IPM
        default_value='can1 can0 pcan0' # order 5G4T, vehicle, and IPM
    )
    return launch.LaunchDescription([
        launch_arg_can_interface, 
        DeclareLaunchArgument('enable_debug', default_value='true'),
        OpaqueFunction(function = launch_setup)
        ])
