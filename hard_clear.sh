#!/bin/bash

cd "$(dirname "$BASH_SOURCE")"

read -rsp "sure?" -n 1 pause1

echo ""

#remove untracked files and folders
git clean -d -ff -x &> /dev/null

#remove ignored tracked files and folders
git ls-files -i --exclude-standard --directory -z| xargs -0 rm -r &> /dev/null

echo "cleared hard"

