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
    
    use_compressed = LaunchConfiguration('use_compressed')
    camera_ns = LaunchConfiguration('camera_ns')
    save_dir = LaunchConfiguration('save_dir')
    device_name = LaunchConfiguration('device_name')

    t_config = ""
    img_encoding = ""
    compressed = False
    if str(use_compressed.perform(context)) == "True" or str(use_compressed.perform(context)) == "true" :
        compressed = True
    if compressed :
        # camera/image_raw/compressed 17HZ
        t_config = "v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1 \
        ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw,format=BGRx ! jpegenc ! jpegparse"
        img_encoding = "jpeg"

        # "v4l2src device=/dev/video0 do-timestamp=true ! video/x-raw,framerate=30/1 \
        #     ! nvvidconv ! video/x-raw(memory:NVMM) ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert \
        #     ! jpegenc ! multipartmux ! multipartdemux ! jpegparse"
    else:
        # 30HZ camera/image_raw/compressed; 5HZ theora
        t_config = "v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1 ! \
        nvvidconv ! video/x-raw(memory:NVMM), format=NV12 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert"
        img_encoding = "rgb8"
    t_dev_name = str(device_name.perform(context))
    gscam_config = t_config.replace("/dev/video0", t_dev_name)

    camera_info_url = "package://gscam/examples/uncalibrated_parameters.ini"
    img_save_path = "./Pictures/camera/"
    cam_ns = context.perform_substitution(camera_ns)
    img_save_path = "".join([context.perform_substitution(save_dir), cam_ns])
    # img_save_path =  os.path.join(save_dir, str(cam_ns.perform(context)))
    img_save_format = ".jpg"
    if not os.path.exists(img_save_path):
        os.makedirs(img_save_path)


    # trigger_cmd = ExecuteProcess(
    #     cmd=[[
    #         'sudo ',
    #         # '-S ',
    #         # '< ',
    #         # '<(echo "nvidia") ',
    #         'tztek-jetson-tool-internal-trigger-camera ',
    #         '/dev/ttyTHS1 ',
    #         '30 ',
    #         '100'
    #     ]],
    #     shell=True
    # )

    gscam_node = ComposableNode(
        package="gscam",
        plugin="gscam::GSCam",
        name="gscam_node",
        namespace=cam_ns,
        parameters=[
            {
                "gscam_config": gscam_config,
                "camera_info_url": camera_info_url,
                "use_gst_timestamps": False,  # 65~70ms
                "image_encoding": img_encoding,
                # "frame_id": "/v4l_frame",
                "sync_sink": True,
                "save_dir": img_save_path,
                "format": img_save_format,
                "use_compressed": compressed,
                "cam_ns": cam_ns,
                'save_flag': LaunchConfiguration('save_flag'),
            }
        ],
        remappings=[
                ("/camera/camera_info", f"{cam_ns}/camera/camera_info"),
                ("/camera/image_raw", f"{cam_ns}/camera/image_raw"),
                (
                    "/camera/image_raw/compressed",
                    f"{cam_ns}/camera/image_raw/compressed",
                ),
                (
                    "/camera/image_raw/compressedDepth",
                    f"{cam_ns}/camera/image_raw/compressedDepth",
                ),
                (
                    "/camera/image_raw/theora",
                    f"{cam_ns}/camera/image_raw/theora",
                ),
            ],
        # Future-proof: enable zero-copy IPC when it is available
        # https://github.com/ros-perception/image_common/issues/212
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    image_convert_node = ComposableNode(
        package="zf_hal_camera_driver",
        plugin="zf::ImageConverter",
        name="hal_camera_driver",
        namespace=cam_ns,
        parameters=[
            {
                "save_dir": img_save_path,
                "format": img_save_format,
                "use_compressed":compressed,
                "cam_ns": cam_ns,
                'save_flag': LaunchConfiguration('save_flag'),
            }
        ]
    )

    return [
            # trigger_cmd,

            ComposableNodeContainer(
                name="gscam_container",
                namespace=cam_ns,
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    
                    # GSCam driver
                    gscam_node,
                    # image_convert_node,
                ],
                output="screen",
                emulate_tty = True,
            )
        ]

def generate_launch_description():

    # Get the path to the user's home directory
    home_dir = os.path.expanduser("~")
    # Define the path to the file you want to save
    file_path = os.path.join(home_dir, "Pictures/camera/")

    launch_arg_use_compressed = DeclareLaunchArgument(
        'use_compressed',
        default_value='False'
    )
    launch_arg_camera_ns = DeclareLaunchArgument(
        'camera_ns',
        default_value='cam1'
    )
    launch_arg_save_dir = DeclareLaunchArgument(
        'save_dir',
        # default_value='~/Pictures/camera/'
        default_value=file_path
    )
    launch_arg_device_name = DeclareLaunchArgument(
        'device_name',
        default_value='/dev/video0'
    )
    launch_arg_img_write_quality = DeclareLaunchArgument(
        'img_write_quality',
        default_value='50'
    )        
    return launch.LaunchDescription([
        launch_arg_use_compressed, 
        launch_arg_camera_ns, 
        launch_arg_save_dir, 
        launch_arg_device_name, 
        launch_arg_img_write_quality,
        DeclareLaunchArgument('save_flag', default_value='0'),
        OpaqueFunction(function = launch_setup)
        ])
