import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('zf_ui_bsdw'),
        'config',
        'params.yaml'
        )
    # wrokspace = os.path.dirname(os.path.dirname(get_package_prefix('zf_ui_bsdw')))
    # # wrokspace = os.environ.get('AMENT_CURRENT_PREFIX')
    # print("wrokspace is" , wrokspace)
    # print("config is" , config)

    node=Node(
        package = 'zf_ui_bsdw',
        name = 'zf_ui_bsdw',
        executable = 'zf_ui_bsdw',
        parameters = [config],
        # parameters=[
        #     {'enable_debug': True}
        # ],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(node)
    return ld