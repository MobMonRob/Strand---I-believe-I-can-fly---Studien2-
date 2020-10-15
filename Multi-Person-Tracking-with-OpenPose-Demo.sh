#!/bin/bash

cd "$(dirname "$BASH_SOURCE")"

# python /home/informatik/git/studienarbeit/i-believe-i-can-fly-v2/python/openpose_webcam_tracking.py
cd /media/informatik/Linux-Daten/openpose
./build/examples/openpose/openpose.bin --face --hand
