#!/bin/bash

CAN_INTERFACE=can0

for DEVICE in /dev/ttyACM*; do
    if [ -e "$DEVICE" ]; then
        echo "Using device: $DEVICE"
        break
    fi
done

if [ -z "$DEVICE" ]; then
    echo "No /dev/ttyACM* device found."
    exit 1
fi

sudo pkill -f slcand
sudo slcand -o -c -s8 "$DEVICE" "$CAN_INTERFACE"
sudo ip link set "$CAN_INTERFACE" up type can bitrate 1000000
ip link show "$CAN_INTERFACE"