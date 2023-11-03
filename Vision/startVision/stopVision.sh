#!/bin/bash

#Verify if the number of arguments is valid
#If no argumens or too many arguments given, throw out error message
if ( [ "$#" -eq 0 ] || [ "$#" -gt 1 ] );
then
    echo "rebootVision: Invalid number of arguments"
#If an invalid number is given, throw out an error message
elif ( [ "$1" -gt 5 ] || [ "$1" -lt 1 ] );
then
    echo "rebootVision: Invalid input"
else

    #Move to check ML Panel
    sleep 1
    xdotool key "alt+Left"
    
    #Stop checkML
    xdotool key "Ctrl+C"

    #Move to runCode Panel
    sleep 1
    xdotool key "alt+Up"
    
    #Stop runCode.sh
    #Yes, it takes a lot of inputs. Why? Because ROS wants to 
    #take a break, that's why
    sleep 1
    xdotool key "Ctrl+C" && xdotool key "Ctrl+C" && xdotool key "Ctrl+C"
    sleep 1
    xdotool type "Z"
    sleep 1
    xdotool key "Ctrl+C"
    sleep 1
    xdotool type "Tab"
    sleep 1
    xdotool key "Ctrl+C"
    
    #Move to openEyes Panel
    sleep 1
    xdotool key "alt+Right"
    #Turn off eyes and kill gscam process
    sleep 1
    xdotool key "Ctrl+C"
    xdotool type "./../turnOffEyes1.sh; kg" && xdotool key Return
    
    #Move back to runCode Panel
    sleep 1
    xdotool key "Alt+Left" 
    #Stop it. Again.
    #ROS: We were on a break!
    sleep 1
    xdotool key "Ctrl+C" && xdotool key "Ctrl+C"
fi