#!/bin/bash
# This script initializes the CAN interface on a Linux system.

echo  Using CAN interface: $1 with $2

if [ ! -e $1 ]; then
    echo "Device $1 does not exist."
    exit 1
fi

bitrate=1000000
sudo slcand -o -c -s8 $1
sudo ip link set $2 type can bitrate $bitrate
sudo ip link set $2 up
sudo ip link show $2