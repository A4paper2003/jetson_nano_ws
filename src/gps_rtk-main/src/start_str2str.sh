#!/bin/bash
# This script starts str2str and handles termination signals

# Function to clean up on termination
cleanup() {
    echo "Stopping str2str..."

    # Try to gracefully stop str2str with SIGTERM
    kill $STR2STR_PID 2>/dev/null

    # Wait for a few seconds to give str2str time to terminate gracefully
    sleep 2

    # If str2str hasn't exited, forcefully terminate it
    if ps -p $STR2STR_PID > /dev/null; then
        echo "str2str is still running. Forcing termination..."
        kill -9 $STR2STR_PID
    fi

    exit 0
}

# Trap SIGINT and SIGTERM signals
trap cleanup SIGINT SIGTERM

# Load parameters from ROS
NTRIP_ADDRESS=$(rosparam get /str2str/ntrip_address)
NTRIP_PORT=$(rosparam get /str2str/ntrip_port)
MOUNTPOINT=$(rosparam get /str2str/mountpoint)
DEVICE=$(rosparam get /str2str/device)
BAUDRATE=$(rosparam get /str2str/baudrate)
DEBUG=$(rosparam get /str2str/debug)

# Validate parameters
if [[ -z "$NTRIP_ADDRESS" || -z "$NTRIP_PORT" || -z "$MOUNTPOINT" || -z "$DEVICE" || -z "$BAUDRATE" ]]; then
    echo "Error: Missing required parameters. Check ROS parameters under /str2str."
    exit 1
fi

# Build the str2str command
STR2STR_CMD="str2str -in ntrip://$NTRIP_ADDRESS:$NTRIP_PORT/$MOUNTPOINT -out serial://$DEVICE:$BAUDRATE:8:n:1 -n 1"

# If debug is false, suppress output to both stdout and stderr
if [ "$DEBUG" = "false" ]; then
    STR2STR_CMD="$STR2STR_CMD > /dev/null 2>&1"
fi

# Start str2str in the background
eval $STR2STR_CMD &
STR2STR_PID=$!

echo "str2str is running with PID $STR2STR_PID"

# Wait for the str2str process to finish
wait $STR2STR_PID

echo "str2str has stopped. Exiting script."
exit 0
