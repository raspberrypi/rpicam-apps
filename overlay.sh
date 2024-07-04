#!/bin/bash

#https://www.raspberrypi.com/documentation/accessories/camera.html#if-you-do-need-to-alter-the-configuration

# Define the path to the config file
CONFIG_FILE="/boot/firmware/config.txt"

# Define the line to be added
LINE_TO_ADD="dtoverlay=ov9281"

# Define the line to be commented
LINE_TO_COMMENT="camera_auto_detect=1"

# Add the line if it does not already exist
if ! grep -q "^$LINE_TO_ADD" "$CONFIG_FILE"; then
    echo "$LINE_TO_ADD" >> "$CONFIG_FILE"
    echo "Added: $LINE_TO_ADD"
else
    echo "Line already exists: $LINE_TO_ADD"
fi

# Comment out the line if it exists and is not already commented
if grep -q "^$LINE_TO_COMMENT" "$CONFIG_FILE"; then
    sed -i "s/^$LINE_TO_COMMENT/#$LINE_TO_COMMENT/" "$CONFIG_FILE"
    echo "Commented out: $LINE_TO_COMMENT"
else
    echo "Line not found or already commented: $LINE_TO_COMMENT"
fi
