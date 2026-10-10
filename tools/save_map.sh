#!/bin/bash

# Set default save directory if no argument is provided
DEFAULT_SAVE_DIR="$HOME/maps/$(date +%Y%m%d_%H%M%S)"
SAVE_DIR="${1:-$DEFAULT_SAVE_DIR}"

# Never treat a pre-existing map as proof that this service call succeeded.
if [ -e "$SAVE_DIR/map.pcd" ] || [ -e "$SAVE_DIR/poses.txt" ]; then
    echo "Error: Existing map files in $SAVE_DIR; choose a new directory" >&2
    exit 1
fi

# Create directory if it doesn't exist
mkdir -p "$SAVE_DIR"

echo "Saving maps to: $SAVE_DIR"

# Call the ROS2 service to save maps
SERVICE_OUTPUT=$(ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: '$SAVE_DIR', save_patches: true}")
SERVICE_STATUS=$?
echo "$SERVICE_OUTPUT"

# Check if the service call was successful
if [ "$SERVICE_STATUS" -eq 0 ] && [[ "$SERVICE_OUTPUT" == *"success=True"* ]] \
   && [ -s "$SAVE_DIR/map.pcd" ] && [ -s "$SAVE_DIR/poses.txt" ]; then
    echo "Maps successfully saved to: $SAVE_DIR"
else
    echo "Error: Failed to save maps"
    exit 1
fi
