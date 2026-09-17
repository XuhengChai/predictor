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
import json

def launch_setup(context, *args, **kwargs):

    # {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc", "filter": ""}
    # 'can0 /home/nvidia/cxh/doc/Can/referDBC/act.dbc'


    # can_json_param = LaunchConfiguration('can_json_param')
    # str_dbc_name = str(can_json_param.perform(context)).split()
    # t_dev_name = str_dbc_name.pop(0)
    can_json_param = LaunchConfiguration('can_json_param')
    str_can_json = str(can_json_param.perform(context))
    dic_can = json.loads(str_can_json)

    t_dev_name = dic_can['name']
    str_dbc_name = dic_can['dbc'].split()

    can_client_node = ComposableNode(
        package="zf_hal_can_driver",
        plugin="zf::driver::canbus::CanDriverNode",
        name="hal_can_driver_node",
        namespace=t_dev_name,
        parameters=[{
            'interface': t_dev_name,
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
            # 'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
            'enable_debug': LaunchConfiguration('enable_debug'),
        }],
        # output='screen',
        # emulate_tty = True,
    )

    can_pp_node = ComposableNode(
        package="zf_hal_canmsg_parser",
        plugin="zf::driver::canbus::CanMsgParserPackNode",
        name="hal_can_pp_node",
        namespace=t_dev_name,
        parameters=[
            {
                "interface": t_dev_name,
                "dbc_files_path": str_dbc_name,
            }
        ],
        # emulate_tty = True,
    )

    return [
            # trigger_cmd,

            ComposableNodeContainer(
                name="can_container",
                namespace=t_dev_name,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    # can_client_node,
                    can_pp_node,
                ],
                output="screen",
                emulate_tty = True,
            )
        ]

def generate_launch_description():

    # Get the path to the user's home directory
    # home_dir = os.path.expanduser("~")
    # Define the path to the file you want to save
    # file_path = os.path.join(home_dir, "Pictures/camera/")
    # can_dir = {"can0": "/home/nvidia/cxh/doc/Can/referDBC/act.dbc", "can1": "/dev/video2"}
    # can_array = '''
    # [
    #     {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc", "filter": ""}
    #     , {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc", "filter": ""}
    # ]
    # '''
    can_array = '''
    {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc"}
    '''

    launch_arg_device_dbc_name = DeclareLaunchArgument(
        'can_json_param',
        default_value=can_array
    )


    return launch.LaunchDescription([
        launch_arg_device_dbc_name, 
        DeclareLaunchArgument('enable_can_fd', default_value='false'),
        DeclareLaunchArgument('timeout_sec', default_value='0.2'),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('enable_debug', default_value='false'),
        OpaqueFunction(function = launch_setup)
        ])
