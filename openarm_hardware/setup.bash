#!/bin/bash
#
# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
