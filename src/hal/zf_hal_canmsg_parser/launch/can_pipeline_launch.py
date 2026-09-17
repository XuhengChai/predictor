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
from launch_ros.actions import LifecycleNode, Node

from launch.actions import (EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os
import json
import re

# def generate_launch_description():

def is_origin_can_name(name):
    pattern = r'^can\d+$'
    return re.match(pattern, name) is not None

def init_a_can_node(str_can_name:str, str_dbc_name_list:list, str_filter:str):
    # TODO: Changable bitrate
    # if is_origin_can_name(str_can_name):
    #     trigger_cmd_down = " ".join(["sudo ip link set down", str_can_name])
    #     trigger_cmd_bitrate = " ".join(["sudo ip link set", str_can_name, "type can bitrate 500000"])
    #     trigger_cmd_up = " ".join(["sudo ip link set up", str_can_name])
    #     # trigger_cmd = 'tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 30 1000'
    #     os.system('echo %s|sudo -S %s' % ('nvidia', trigger_cmd_down)) 
    #     os.system('sudo -S %s' % (trigger_cmd_bitrate)) 
    #     os.system('sudo -S %s' % (trigger_cmd_up))
    #     print(trigger_cmd_down)
    #     print(trigger_cmd_bitrate)
    #     print(trigger_cmd_up)

    nodes = []
    can_client_node = LifecycleNode(
        package='zf_hal_can_driver',
        executable='hal_can_driver_node_exe',
        name='hal_can_driver',
        namespace=str_can_name,
        parameters=[{
            'interface': str_can_name,
            'baudrate': LaunchConfiguration('baudrate'),
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
            'filters': str_filter,
            'use_bus_time': LaunchConfiguration('use_bus_time'),
            'enable_debug': LaunchConfiguration('enable_debug_raw'),
            # 'enable_debug': True,
            'log_cnt': LaunchConfiguration('log_cnt'),
        }],
        output='screen',
        emulate_tty = True,
        )
    can_driver_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=can_client_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(can_client_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    can_driver_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=can_client_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(can_client_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    can_pp_node = Node(
        package="zf_hal_canmsg_parser",
        executable="hal_canmsg_pp_node_exe",
        name="hal_can_pp_node",
        namespace=str_can_name,
        parameters=[
            {
                "interface": str_can_name,
                "can_description": LaunchConfiguration('can_description'),
                "dbc_files_path": str_dbc_name_list,
                'log_cnt': LaunchConfiguration('log_cnt'),
                'enable_debug': LaunchConfiguration('enable_debug'),
                'vehicle_can_interface': LaunchConfiguration('vehicle_can_interface'),
            }
        ],
        output="screen",
        emulate_tty = True,
        # emulate_tty = True,
    )
    nodes.extend([
        can_client_node,
        can_driver_configure_event_handler,
        can_driver_activate_event_handler,
        can_pp_node,
        ])
    return nodes

def launch_setup(context, *args, **kwargs):
    can_json_param = LaunchConfiguration('can_json_param')
    # vehicle_can_name = LaunchConfiguration('vehicle_can_name')
    str_can_json = str(can_json_param.perform(context))
    # str_vehicle_can_name = str(vehicle_can_name.perform(context))
    dic_can = json.loads(str_can_json)
    can_number = len(dic_can)
    nodes = []
    # enable_debug_raw = str(LaunchConfiguration('enable_debug_raw').perform(context))
    # enable_debug = str(LaunchConfiguration('enable_debug').perform(context))
    # print( "------------------------", enable_debug_raw)
    # print( "------------------------", enable_debug)
    for item in dic_can:
        str_can_name = item['name']
        str_dbc_name = item['dbc'].split()
        str_filter = item['filter']
        nodes.extend(init_a_can_node(str_can_name, str_dbc_name, str_filter))
    return nodes


def generate_launch_description():
    # Get the path to the user's home directoryfalse
    # Define the path to the file you want to save
    # file_path = os.path.join(home_dir, "Pictures/camera/")
    # can_dir = {"can0": "/home/nvidia/cxh/doc/Can/referDBC/act.dbc", "can1": "/dev/video2"}
    # {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc", "filter": "0:0"}
    # {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc", "filter": "65D:7FF,718:7FF"}
        # , {"name": "can1", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/can_ARS408_conti.dbc", "filter": "0:0"}

    # can_array = '''
    # [
    #     {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/referDBC/GB_can_nova.dbc /home/nvidia/cxh/doc/Can/referDBC/VehicleCAN.dbc", 
    #     "filter": "FE6E00:FFFF00,FE6C00:FFFF00,FEF100:FFFF00,F00900:FFFF00,F00100:FFFF00,F00200:FFFF00,F00500:FFFF00,F00400:FFFF00,F00300:FFFF00,FEDF00:FFFF00,500:FFFF00,600:FFFF00,700:FFFF00"}
    # ]
    # '''

    #  vehiclecan DBC fisrt
    can_array = '''
    [
        {"name": "can0", "dbc": "/home/nvidia/cxh/doc/Can/DBC/VehicleCAN.dbc /home/nvidia/cxh/doc/Can/DBC/FFS1_CAN_J1939_ac1000t.dbc", "filter": "0:0"}
    ]
    '''
    # can_array = '''
    # [
    #     {"name": "pcan0", "dbc": "/home/nvidia/cxh/doc/Can/DBC/ZF_Fusion_IPMoutput.dbc", 
    #     "filter": "0:0"}
    # ]
    # '''

    # {"name": "can1", "dbc": "/home/nvidia/cxh/doc/Can/DBC/VehicleCAN.dbc", 
    # can_array = '''
    # [
    #     {"name": "can1", "dbc": "/home/nvidia/cxh/doc/Can/DBC/VCOM_5G4T.dbc", 
    #     "filter": "0:0"}
    # ]
    # '''
    launch_arg_can_json = DeclareLaunchArgument(
        'can_json_param',
        default_value=can_array
    )
    return launch.LaunchDescription([
        launch_arg_can_json, 
        DeclareLaunchArgument('can_description', default_value='0'),
        DeclareLaunchArgument('vehicle_can_interface', default_value='can0'),
        DeclareLaunchArgument('baudrate', default_value='1000'),
        DeclareLaunchArgument('enable_can_fd', default_value='false'),
        DeclareLaunchArgument('timeout_sec', default_value='0.2'),
        DeclareLaunchArgument('log_cnt', default_value='0'),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('enable_debug', default_value='true'),
        DeclareLaunchArgument('enable_debug_raw', default_value='true'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        OpaqueFunction(function = launch_setup)
        ])


