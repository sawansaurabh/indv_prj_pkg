#!/bin/bash

# Define the tmux session name
SESSION_NAME="ros_session"

# Define your ROS commands
WORKSPACE_PATH="/home/sawan/ros2_ws"

CMD1="source $WORKSPACE_PATH/install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint"

CMD2="ros2 run image_transport republish compressed raw --ros-args -r __node:=im_transport_rgb -r in/compressed:=/camera/rgb/image_raw/compressed -r out:=/camera/rgb/image_raw"

CMD3="ros2 run image_transport republish compressed raw --ros-args -r __node:=im_transport_depth -r in/compressed:=/camera/depth/image_raw/compressed -r out:=/camera/depth/image"

CMD4="source $WORKSPACE_PATH/install/setup.bash && ros2 run indv_prj_pkg depthConverter"

CMD5="source $WORKSPACE_PATH/install/setup.bash && ros2 launch indv_prj_pkg myrtabmap.launch.py"

CMD6="source $WORKSPACE_PATH/install/setup.bash && ros2 run indv_prj_pkg tf_broadcaster"


# Start a new tmux session
tmux new-session -d -s $SESSION_NAME

# Split the window into panes and run the commands
tmux send-keys -t $SESSION_NAME:0.0 "$CMD1" C-m

tmux split-window -v -p 70 -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME:0.1 "$CMD2" C-m

tmux split-window -v -p 70 -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME:0.2 "$CMD3" C-m

tmux split-window -v -p 70 -t $SESSION_NAME:0.2
tmux send-keys -t $SESSION_NAME:0.3 "$CMD4" C-m

tmux split-window -v -p 70 -t $SESSION_NAME:0.3
tmux send-keys -t $SESSION_NAME:0.4 "$CMD5" C-m

tmux split-window -v -p 70 -t $SESSION_NAME:0.4
tmux send-keys -t $SESSION_NAME:0.5 "$CMD6" C-m

# Attach to the tmux session
tmux attach -t $SESSION_NAME

