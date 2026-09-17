gnome-terminal --tab -- bash -c "echo nvidia|sudo -S tztek-jetson-tool-internal-trigger-camera /dev/ttyTHS1 30 1000 & echo 'Camera triggered!';sleep 10" 

pkill -f "ros2 launch zf_hal_sync sync_pipeline_with_2cam.py"
pkill -f "ros2 run zf_detect_tracking detect_tracking"
pkill -f "ros2 run fusion node_fusion"
pkill -f "ros2 run zf_traj_prediction traj_prediction"
pkill -f "./zf_io_exe"

sleep 2
gnome-terminal --tab -- bash -c "ros2 launch zf_hal_sync sync_pipeline_with_2cam.py;"
gnome-terminal --tab -- bash -c "ros2 run zf_detect_tracking detect_tracking;"
gnome-terminal --tab -- bash -c "ros2 run fusion node_fusion;"
gnome-terminal --tab -- bash -c "ros2 run zf_traj_prediction traj_prediction;"
echo 'all start!'
