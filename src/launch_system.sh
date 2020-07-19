#!/bin/bash

# ONLY USE THIS AT STARTUP, IT SHUTS DOWN ALL ACTIVE ROS NODES AND TMUX SESSIONS

session="marble_uav_session"

tmux set -g history-limit 10000

tmux new-session -s $session -d

tmux set -g mouse on
tmux send-keys -t 0 'oneflight' Enter

tmux -2 attach-session -t $session -d
