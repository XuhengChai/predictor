install requirement:
sudo apt-get install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev
sudo apt-get install ros-foxy-camera-calibration-parsers
sudo apt-get install ros-foxy-camera-info-manager
sudo apt-get install ros-foxy-image-transport-plugins


1. launch the single_camera_pipeline_launch by:
ros2 launch zf_hal_camera_driver single_camera_pipeline_launch.py use_compressed:=False

This is the defult params:
use_compressed:'True'
camera_ns:'cam1'
save_dir: In home dir, ~/Pictures/camera/
device_name:'/dev/video0'

2. If you want to launch more than one camera:
ros2 launch zf_hal_camera_driver camera_pipeline_launch.py

you can change the camera_dir as you own defines:
camera_dir = {"cam1": "/dev/video0", "cam2": "/dev/video2"} 
then rebuild:
colcon build --package-select zf_hal_camera_driver

ros2 launch zf_hal_camera_driver camera_pipeline_launch.py
ros2 launch zf_hal_sync sync_pipeline_with_2cam.py
ros2 topic echo /cam2/node_state
ros2 topic echo /cam1/node_state
ros2 topic echo /can_state

#!/bin/bash
while :
do
	source install/setup.bash
	gnome-terminal --tab -- bash -c "echo nvidia|sudo -S ./can0_can1fd.sh; sleep 3"
	gnome-terminal --tab -- bash -c "timeout 55 ros2 launch zf_hal_canmsg_parser mutican_pipeline_launch.py; sleep 1;"
	sleep 3
	gnome-terminal --tab -- bash -c "timeout 55 ros2 launch zf_hal_sync sync_pipeline_with_2cam.py; sleep 1;"
	echo '5s configurated!'
	sleep 60
done

killall repeat5min.sh

#!/bin/bash
pkill -f "ros2 launch zf_hal_canmsg_parser mutican_pipeline_launch.py"
pkill -f "ros2 run ego_state node_es"
pkill -f "ros2 launch zf_hal_sync sync_pipeline_with_2cam.py"
pkill -f "ros2 run zf_detect_tracking detect_tracking"
pkill -f "ros2 run zf_ui_bsdw zf_ui_bsdw"
pkill -f "./zf_io_exe"

sleep 1
gnome-terminal --tab -- bash -c "ros2 launch zf_hal_canmsg_parser mutican_pipeline_launch.py;"
gnome-terminal --tab -- bash -c "ros2 run ego_state node_es;"
gnome-terminal --tab -- bash -c "./zf_io_exe;"
sleep 2
gnome-terminal --tab -- bash -c "ros2 launch zf_hal_sync sync_pipeline_with_2cam.py;"
gnome-terminal --tab -- bash -c "ros2 run zf_detect_tracking detect_tracking;"
gnome-terminal --tab -- bash -c "ros2 run zf_ui_bsdw zf_ui_bsdw;"
echo 'all start!'