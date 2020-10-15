#!/bin/bash

cd $(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

gnome-terminal -x bash -i -c "./Multi-Person-Tracking-with-OpenPose-Demo.sh; exec bash"


