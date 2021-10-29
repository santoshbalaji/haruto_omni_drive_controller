#!/bin/bash

source_workspace="source /home/ubuntu/catkin_ws/devel/setup.bash"

tmux has-session -t haruto
if [ $? != 0 ]; then
	tmux new-session -s haruto -n haruto -d

	tmux set -g mouse on
	tmux set -s status-interval 0
	tmux set -g pane-border-status top

	tmux send-keys -t haruto "$source_workspace" C-m
	tmux send-keys "roslaunch haruto_controller haruto_controller.launch" C-m
	tmux select-layout tiled

	tmux split-window -v -t haruto
	tmux send-keys -t haruto "$source_workspace" C-m
	tmux send-keys "roslaunch haruto_controller haruto_arduino.launch" C-m
	tmux select-layout tiled

	tmux split-window -v -t haruto
	tmux send-keys -t haruto "$source_workspace" C-m
	tmux send-keys "roslaunch haruto_controller haruto_simulation.launch" C-m
	tmux select-layout tiled

	tmux select-pane -t 0 -T controller
	tmux select-pane -t 1 -T arduino
	tmux select-pane -t 2 -T teleop

fi 
