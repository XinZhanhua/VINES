#!/bin/sh

# message_transformer
gnome-terminal --tab -t "tab1" -e "bash -c 'roscore;'" 
sleep 5
gnome-terminal --tab -t "tab2" -e "bash -c 'source ~/vines_ws/devel/setup.bash; roslaunch vines_estimator gimbal.launch;'" --tab -t "tab3" -e "bash -c 'source ~/vins_ws/devel/setup.bash; roslaunch vins_estimator vins_rviz.launch;'"
