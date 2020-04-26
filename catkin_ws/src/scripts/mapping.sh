#!/bin/sh

xterm -e " roslaunch my_robot world.launch" &
sleep 5
xterm -e " roslaunch teleop_twist_keyboard teleop.launch" &
sleep 5
xterm -e " roslaunch my_robot mapping.launch" &
