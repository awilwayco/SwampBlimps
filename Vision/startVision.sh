#!/bin/bash

# Start Terminator
cd ~/Vision
terminator &
sleep 0.5

# Split the terminal vertically
xdotool key ctrl+shift+e

# Split the terminal horizontally
xdotool key ctrl+shift+o
xdotool type 'cd Record && clear'
xdotool key Return

xdotool key alt+Left

# Split the terminal horizontally
xdotool key ctrl+shift+o
xdotool type 'cd Viewers && clear'
xdotool key Return

xdotool key alt+Up

if [ "$1" -eq 1 ]; then
    echo $#
    xdotool type './runVision1.sh'
    xdotool key Return
    xdotool key alt+Right
    sleep 14
    xdotool type './turnOnEyes1.sh'
    xdotool key Return
    xdotool key alt+Down
    xdotool key alt+Left
elif [ "$1" -eq 2 ]; then
    xdotool type './runVision2.sh'
    xdotool key Return
    xdotool key alt+Right
    sleep 14
    xdotool type './turnOnEyes2.sh'
    xdotool key Return
    xdotool key alt+Down
    xdotool key alt+Left
elif [ "$1" -eq 3 ]; then
    xdotool type './runVision3.sh'
    xdotool key Return
    xdotool key alt+Right
    sleep 14
    xdotool type './turnOnEyes3.sh'
    xdotool key Return
    xdotool key alt+Down
    xdotool key alt+Left
elif [ "$1" -eq 4 ]; then
    xdotool type './runVision4.sh'
    xdotool key Return
    xdotool key alt+Right
    sleep 14
    xdotool type './turnOnEyes4.sh'
    xdotool key Return
    xdotool key alt+Down
    xdotool key alt+Left
elif [ "$1" -eq 5 ]; then
    xdotool type './runVision5.sh'
    xdotool key Return
    xdotool key alt+Right
    sleep 14
    xdotool type './turnOnEyes5.sh'
    xdotool key Return
    xdotool key alt+Down
    xdotool key alt+Left
else
    echo "Invalid Argument: The first argument is not between 1 and 5."
fi

