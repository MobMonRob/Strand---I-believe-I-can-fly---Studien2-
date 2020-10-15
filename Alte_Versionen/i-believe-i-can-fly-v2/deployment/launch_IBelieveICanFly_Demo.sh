#!/bin/bash

cd $(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

gnome-terminal -x bash -i -c "./I-believe-I-can-fly-Demo.sh; exec bash"


