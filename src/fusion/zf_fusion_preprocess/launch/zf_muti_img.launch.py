from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zf_fusion_preprocess',
            namespace='turtlesim1',
            executable='img_preprocess',
            name='sim'
        ),
        Node(
            package='zf_fusion_preprocess',
            namespace='turtlesim2',
            executable='img_preprocess',
            name='sim'
        ),
        Node(
            package='zf_fusion_preprocess',
            namespace='turtlesim3',
            executable='img_preprocess',
            name='sim'
        )
    ])
