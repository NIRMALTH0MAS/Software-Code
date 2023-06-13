#!/bin/bash

# Launch first Python program in a new terminal
gnome-terminal --tab --title="Program 1" --command="bash -c 'cd $(dirname "$0"); python3 Images_Save_multicamera1.py; $SHELL'"

# Launch second Python program in a new terminal
gnome-terminal --tab --title="Program 2" --command="bash -c 'cd $(dirname "$0"); python3 Images_Save_multicamera2.py; $SHELL'"
