#!/bin/bash

cd $(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

/home/informatik/'Unreal Environments'/AirSimNH/AirSimNH.sh -ResX=1920 -ResY=1050 -windowed &
roslaunch ../src/catkin/i-believe-i-can-fly-showcase.launch && wait


