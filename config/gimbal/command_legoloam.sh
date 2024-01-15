#!/bin/sh

# message_transformer
gnome-terminal --tab -t "tab1" -e "bash -c 'roscore;'" 
sleep 5
gnome-terminal --tab -t "tab2" -e "bash -c 'source ~/vines_ws/devel/setup.bash; roslaunch lego_loam run.launch;'" --tab -t "tab3" -e "clear"
