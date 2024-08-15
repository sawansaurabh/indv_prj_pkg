#!/bin/bash

# Define the tmux session name
SESSION_NAME="ros_session"

# Send SIGINT (Ctrl+C) to all panes to stop running processes
tmux send-keys -t $SESSION_NAME:0 C-c
tmux send-keys -t $SESSION_NAME:0.1 C-c
tmux send-keys -t $SESSION_NAME:0.2 C-c
tmux send-keys -t $SESSION_NAME:0.3 C-c
tmux send-keys -t $SESSION_NAME:0.4 C-c
tmux send-keys -t $SESSION_NAME:0.5 C-c

# Give processes some time to shut down gracefully
sleep 1

# Kill the tmux session
tmux kill-session -t $SESSION_NAME

