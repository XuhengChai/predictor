pkill -f "ros2 launch zf_hal_sync sync_pipeline_with_2cam.py"
pkill -f "ros2 launch zf_hal_canmsg_parser mutican_pipeline_launch.py"
pkill -f "ros2 run zf_detect_tracking detect_tracking"
pkill -f "ros2 run ego_state node_es"
pkill -f "ros2 run fusion node_fusion"
pkill -f "ros2 run zf_traj_prediction traj_prediction"
pkill -f "./zf_io_exe"

echo 'all end!'
