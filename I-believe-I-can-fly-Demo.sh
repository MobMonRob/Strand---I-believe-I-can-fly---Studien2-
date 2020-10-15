#!/bin/bash

cd "$(dirname "$BASH_SOURCE")"

/home/informatik/'Unreal Environments'/AirSimNH/AirSimNH.sh -ResX=1920 -ResY=1050 -windowed &
roslaunch ./src/catkin/i-believe-i-can-fly-showcase.launch && wait

