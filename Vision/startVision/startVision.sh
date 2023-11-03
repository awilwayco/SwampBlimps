#!/bin/bash

#Verify if the number of arguments is valid
#If no argumens or too many arguments given, throw out error message
if ( [ "$#" -eq 0 ] || [ "$#" -gt 1 ] );
then
    echo "bootUpVision: Invalid number of arguments"
#If an invalid number is given, throw out an error message
elif ( [ "$1" -gt 5 ] || [ "$1" -lt 1 ] );
then
    echo "bootUpVision: Invalid input"; clear

else

    #Pings Pi
    num1=$(ping -c 1 192.168.0.11$1 | tail -n 5 | head -n 1 | egrep "ttl=" | wc -l)

    #num2=$(./../Viewers/checkML1.sh | egrep "nanosec: 0" | wc -l)

    #Run Vision Code
    xdotool type "./../runVision$1.sh"
    xdotool key Return
    
    #Move to Ping Screen
    sleep 1
    xdotool key "alt+Right"

    #Ping Pi, wait until Pi message is received and then open
    #the Pi's eyes (i.e. turn on the camera connected to the Pi)
    xdotool type "sleep 7; while [ \"$num1\" -ne 1 ]; do echo \"Pi not Found\"; sleep 1; done; ./../turnOnEyes$1.sh"
    xdotool key Return

    #Move to Check ML Panel
    sleep 1
    xdotool key "alt+Down"
    xdotool key "alt+Left"
    
    #Wait until Pi message is received and after ./openEyes of the
    #Pi has been run before running ./checkML of the Pi
    xdotool type "sleep 7; while [ \"$num1\" -ne 1 ]; do echo \"Pi not Found\"; sleep 1; done; ./../Viewers/checkML$1.sh"
    xdotool key Return

    #Move to user panel 
    sleep 1
    xdotool key "alt+Right"
    #Enter Viewers Folder
    xdotool type "cd ../Viewers; clear" && xdotool key Return

fi