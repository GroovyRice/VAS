#!/bin/bash

# Kill all existing tmux sessions
tmux kill-server

# Allow connections from Docker containers to the X server
xhost +

# Start Docker Compose
docker-compose up -d

# Wait for the containers to be up
sleep 3

# Create a new tmux session
tmux new-session -d -s ros2_services

# Split the window into three panes
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Run commands in each pane

# Pane 0: depthai
tmux send-keys -t ros2_services:0.0 "docker exec -it docker_depthai_1 /bin/bash" C-m
tmux send-keys -t ros2_services:0.0 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t ros2_services:0.0 "ros2 launch depthai_examples tracker_yolov4_spatial_node.launch.py" C-m

# Pane 1: tfmini
tmux select-pane -t 1
tmux send-keys -t ros2_services:0.1 "docker exec -it docker_tfmini_1 /bin/bash" C-m
tmux send-keys -t ros2_services:0.1 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t ros2_services:0.1 "cd /vas" C-m
tmux send-keys -t ros2_services:0.1 "colcon build" C-m
tmux send-keys -t ros2_services:0.1 "source install/setup.bash" C-m
tmux send-keys -t ros2_services:0.1 "ros2 run tfmini tfmini_node" C-m

# Pane 2: vas
tmux select-pane -t 2
tmux send-keys -t ros2_services:0.2 "docker exec -it docker_vas_1 /bin/bash" C-m
tmux send-keys -t ros2_services:0.2 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t ros2_services:0.2 "cd /vas" C-m
tmux send-keys -t ros2_services:0.2 "colcon build" C-m
tmux send-keys -t ros2_services:0.2 "source install/setup.bash" C-m
tmux send-keys -t ros2_services:0.2 "ros2 run subscribers detections_node" C-m

# Pane 3: tts
tmux select-pane -t 3
tmux send-keys -t ros2_services:0.3 "docker exec -it docker_tts_1 /bin/bash" C-m
tmux send-keys -t ros2_services:0.3 "source /opt/ros/humble/setup.bash" C-m
tmux send-keys -t ros2_services:0.3 "cd /vas" C-m
tmux send-keys -t ros2_services:0.3 "colcon build" C-m
tmux send-keys -t ros2_services:0.3 "source install/setup.bash" C-m
tmux send-keys -t ros2_services:0.3 "ros2 run tts tts_node" C-m

# Attach to the tmux session
tmux attach-session -t ros2_services
