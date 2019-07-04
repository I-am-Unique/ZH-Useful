#!/bin/bash
source /opt/ros/kinetic/setup.sh
source /home/qingzhi/jingmabus/devel/setup.bash
rosclean purge -y
# roscore
# gnome-terminal

roslaunch /home/qingzhi/jingmabus/src/decision/launch/allinone.launch

