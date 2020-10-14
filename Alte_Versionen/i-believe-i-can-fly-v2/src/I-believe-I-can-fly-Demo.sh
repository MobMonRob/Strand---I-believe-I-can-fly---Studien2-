#!/bin/bash

/home/informatik/'Unreal Environments'/AirSimNH/AirSimNH.sh -ResX=1920 -ResY=1050 -windowed &
roslaunch /home/informatik/git/studienarbeit/i-believe-i-can-fly-v2/catkin/i-believe-i-can-fly-showcase.launch &&
wait

