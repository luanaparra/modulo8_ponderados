#!/bin/bash

# Function to run when Ctrl+C is pressed
save_map() {
    ros2 run nav2_map_server map_saver_cli -f map
    # Add your cleanup command here
    # For example: kill some_process
    wait
    # Wait for the background process to finish before exiting
    exit 0
}

# Set the cleanup function to be called on Ctrl+C
trap save_map SIGINT

# Your main command goes here
ros2 launch launch/map_launch.py &

wait 
