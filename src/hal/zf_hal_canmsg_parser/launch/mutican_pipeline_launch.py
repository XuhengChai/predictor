from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    launchs = []
    # ros2 launch zf_hal_camera_driver camera_pipeline_launch.py

    # t_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             PathJoinSubstitution([
    #                 FindPackageShare('zf_hal_camera_driver'),
    #                 'launch',
    #                 'camera_pipeline_launch.py'
    #             ])
    #         ])
    #     )
    # launchs.append(t_launch)

    #  "filter": "FE6E00:FFFF00,FE6C00:FFFF00,FEF100:FFFF00,F00900:FFFF00,F00100:FFFF00,F00200:FFFF00,F00500:FFFF00,F00400:FFFF00,F00300:FFFF00,FEDF00:FFFF00,500:FFFF00,600:FFFF00,700:FFFF00"}

    # filters refers to https://manpages.debian.org/testing/can-utils/candump.1.en.html
    # right save 60 61
    # FDCC turning lights
    # can_array0 = '''
    # [
    #     {"name": "can0", "dbc": "/home/nvidia/gitlab/trailerATA/config/dbc/VehicleCAN.dbc", 
    #     "filter": "FE6E00:FFFF00,FE6C00:FFFF00,FEF100:FFFF00,F00900:FFFF00,F00100:FFFF00,F00200:FFFF00,F00500:FFFF00,
    #     F00400:FFFF00,F00300:FFFF00,FEDF00:FFFF00,FDCC00:FFFF00,FEBF00:FFFF00,
    #     500:FFFF00,600:FFFFF0,610:FFFFF0,700:FFFF00,
    #     000000:FF00000,040000:FF00000,F00700:FFFF00,EB0000:FF00000,EC0000:FF00000"}
    # ]
    # '''

    can_array0 = '''
    [
        {"name": "can0", "dbc": "/home/nvidia/gitlab/trailerATA/config/dbc/VehicleCAN.dbc /home/nvidia/gitlab/trailerATA/config/dbc/FFS1_CAN_J1939_ac1000t.dbc", 
        "filter": "0:0"}
    ]
    '''
    # left save 64 65
    can_array1 = '''
    [
        {"name": "can1", "dbc": "/home/nvidia/gitlab/trailerATA/config/dbc/VCOM_5G4T.dbc", 
        "filter": "0:0"}
    ]
    '''
    can_array2 = '''
    [
        {"name": "pcan0", "dbc": "/home/nvidia/gitlab/trailerATA/config/dbc/ZF_Fusion_IPMoutput.dbc", 
        "filter": "0:0"}
    ]
    '''
    # vehicle_can_name:str = "can0"
    # camera_dir = {"cam2": "/dev/video2"}
    CAN_DES_VEHICLE = "0"
    CAN_DES_5G4T = "1"
    CAN_DES_IPM = "2"
    CAN_DES_AC1000T = "0"

    t_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zf_hal_canmsg_parser'),
                    'launch',
                    'can_pipeline_launch.py'
                ])
            ]),
            launch_arguments={
                'can_json_param': can_array0,
                'can_description': CAN_DES_VEHICLE,
                'log_cnt': '0',
                'enable_debug': 'false',
                'enable_debug_raw': 'false'
            }.items()
        )
    launchs.append(t_launch)


    t_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zf_hal_canmsg_parser'),
                    'launch',
                    'can_pipeline_launch.py'
                ])
            ]),
            launch_arguments={
                'can_json_param': can_array1,
                'can_description': CAN_DES_5G4T,
                'baudrate': '1000',
                'log_cnt': '1',
                'enable_can_fd': 'true',
                'enable_debug': 'false',
                'enable_debug_raw': 'false'
            }.items()
        )
    launchs.append(t_launch)

    t_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zf_hal_canmsg_parser'),
                    'launch',
                    'can_pipeline_launch.py'
                ])
            ]),
            launch_arguments={
                'can_json_param': can_array2,
                'can_description': CAN_DES_IPM,
                'baudrate': '1000',
                'log_cnt': '2',
                'enable_can_fd': 'false',
                'use_bus_time': 'true',
                'enable_debug': 'false',
                'enable_debug_raw': 'false'
            }.items()
        )
    launchs.append(t_launch)

    return LaunchDescription(launchs)
