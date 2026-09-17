# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.


from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    hal_can_driver_node = LifecycleNode(
        package='zf_hal_can_driver',
        executable='hal_can_driver_node_exe',
        name='hal_can_driver',
        namespace=TextSubstitution(text='eth4'),
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'timeout_sec': LaunchConfiguration('timeout_sec'),
            'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
            'enable_debug': LaunchConfiguration('enable_debug'),
            'log_cnt': LaunchConfiguration('log_cnt'),
            'baudrate': LaunchConfiguration('baudrate'),
        }],
        output='screen',
        emulate_tty = True,
        )

    hal_can_driver_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=hal_can_driver_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hal_can_driver_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure_driver')),
    )

    hal_can_driver_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=hal_can_driver_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(hal_can_driver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate_driver')),
    )

    # 2C1:7FF,65d:7ff
    return LaunchDescription([
        DeclareLaunchArgument('interface', default_value='eth4'),
        DeclareLaunchArgument('enable_can_fd', default_value='false'),
        DeclareLaunchArgument('timeout_sec', default_value='0.2'),
        DeclareLaunchArgument('log_cnt', default_value='0'),
        DeclareLaunchArgument('baudrate', default_value='1000'),
        DeclareLaunchArgument('use_bus_time', default_value='false'),
        DeclareLaunchArgument('enable_debug', default_value='true'),
        DeclareLaunchArgument('filters', default_value='0:0',
                              description='Comma separated filters can be specified for each given'
                                          ' CAN interface.\n'
                                          '\t<can_id>:<can_mask>\n'
                                          '\t\t(matches when <received_can_id> & mask == can_id & '
                                          'mask)\n'
                                          '\t<can_id>~<can_mask>\n'
                                          '\t\t(matches when <received_can_id> & mask != can_id & '
                                          'mask)\n'
                                          '\t#<error_mask>\n'
                                          '\t\t(set error frame filter, see include/linux/can/'
                                          'error.h)\n'
                                          '\t[j|J]\n'
                                          '\t\t(join the given CAN filters - logical AND '
                                          'semantic)\n\n'
                                          '\tCAN IDs, masks and data content are given and '
                                          'expected in hexadecimal values. When can_id and '
                                          'can_mask are both 8 digits, they are assumed to '
                                          "be 29 bit EFF. '0:0' default filter will accept "
                                          'all data frames.\n'
                                          '\tFor more information about syntax check: '
                                          'https://manpages.ubuntu.com/manpages/jammy/'
                                          'man1/candump.1.html'),
        DeclareLaunchArgument('auto_configure_driver', default_value='true'),
        DeclareLaunchArgument('auto_activate_driver', default_value='true'),
        hal_can_driver_node,
        hal_can_driver_configure_event_handler,
        hal_can_driver_activate_event_handler,
    ])
