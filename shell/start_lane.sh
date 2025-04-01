#!/bin/bash

# START


screen -ls | grep driver
screen -ls | grep lane

if ! screen -ls | grep -q "lane"
then
    echo -e "\e[42mStart lane\e[0m"
    screen -m -d -S lane bash -c 'source ~/ros2_ws/install/setup.bash && ros2 launch lane_following_cam robot_compressed1.launch.py multiplier_bottom:=1.0 multiplier_top:=0.65 divisor:=5.0 islane:=false'
else
    echo -e "\e[41merror\e[0m lane already started"
fi
chmod +x ~/ros2_ws/src/savtartas/shell/start_lane.sh
alias start_lane='~/ros2_ws/src/savtartas/shell/start_lane.sh'

