#!/bin/bash

IP=$(hostname -I | awk '{print $1}')
URL="http://$IP:5000"

echo ""
echo "Basestation Starting..."
echo ""
echo "Running on $URL"
echo ""
echo "Getting Logs..."
echo ""

./freePort.sh

# For Bounded Box Custom Messages
#source $CATCHING_BLIMP_PATH/Summer2023/ros2_stereo/install/setup.bash

# Use -o flag to open website and run the program
if [ "$1" == "-o" ]; then
    xdg-open "$URL" > /dev/null 2>&1;
    python3 main.py $IP
# No flags just runs the programs
else
    python3 main.py $IP
fi