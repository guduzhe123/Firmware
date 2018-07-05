#!/usr/bin/env zsh
## Firmware V1.7
source ~/ros1_ws/src/WindAppCore/data/scripts/sync_gazebo_wind_turbine_model.sh
make all
mkdir -p /tmp/frames
source ~/ros1_ws/devel/setup.zsh
source Tools/setup_gazebo.zsh $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch

#sleep 10s
#roslaunch px4_ctrl vel_ctrl_plus_dji.launch