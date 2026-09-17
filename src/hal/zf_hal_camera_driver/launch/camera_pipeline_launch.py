from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import os
import json


def generate_launch_description():
    cam_array = '''
    [
        {"camera_ns": "cam1", "device_name": "/dev/video0", "save_flag": "0",
        "use_compressed": "False", "img_write_quality": "50"},
        {"camera_ns": "cam2", "device_name": "/dev/video2", "save_flag": "0",
        "use_compressed": "False", "img_write_quality": "50"}
    ]
    '''
    # cam_array = '''
    # [
    #     {"camera_ns": "cam1", "device_name": "/dev/video0", "save_flag": "1",
    #     "use_compressed": "False", "img_write_quality": "50"}
    # ]
    # '''
    cam_list = json.loads(cam_array)
    # Get the path to the user's home directory
    home_dir = os.path.expanduser("~")
    # Define the path to the file you want to save
    folder_path = os.path.join(home_dir, "Music/log/")
    file_path = os.path.join(folder_path, "logtime.txt")
    with open(file_path, 'r') as file:
        folder_path = file.readline().strip() + "/"
    print(folder_path)
    launchs = []
    cam_name_list = []
    str_imgw_quality = ""
    for item in cam_list:
        str_cam_ns = item['camera_ns']
        cam_name_list.append(str_cam_ns)
        str_device_name = item['device_name']
        str_use_compressed = item['use_compressed']
        save_flag = item['save_flag']
        str_imgw_quality = item['img_write_quality']

        t_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('zf_hal_camera_driver'),
                    'launch',
                    'single_camera_pipeline_launch.py'
                ])
            ]),
            launch_arguments={
                'camera_ns': str_cam_ns,
                'device_name': str_device_name,
                'use_compressed': str_use_compressed,
                'img_write_quality': str_imgw_quality,
                'save_flag': save_flag,
                'save_dir': folder_path,
            }.items()
        )
        launchs.append(t_launch)
    return LaunchDescription(launchs)
