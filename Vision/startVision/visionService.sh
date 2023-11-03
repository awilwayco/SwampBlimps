#/bin/bash

#Verify flags
#If -a flag is sent, start all Pi's
if ( [ "$#" -eq 1 ] && [ "$1" == "-a" ] );
then    
    for currVal in {1..5}
    do
        #Create screens
        ./createScreens.sh
        
        #Boot Up Vision
        ./startVision.sh $currVal
    done
#If -n flag is sent, start all Pi's user has inputed
elif ( [ "$#" -gt 1 ] && [ "$1" == "-n" ] );
then
    #Skip the first command line argument (i.e. -n)
    shift

    #For all command-line arguments after the -n flag
    for currVal in $@
    do
        #Ensure it's a valid number, otherwise throw error message
        if ( [ ${currVal} -gt 0 ] && [ ${currVal} -lt 6 ] );
        then
            #Create screens
            ./createScreens.sh

            #Boot Up Vision
            ./startVision.sh $currVal
        else
            echo "visionService: Invalid value"
        fi
    done
#Either the user has sent the wrong flag, or the wrong number of arguments
else
    echo "Invalid Flag or Invalid Number of Arguments"
fi


#Testing code (used for development and testing various concepts)

#Create screens
# ./createScreens.sh

#Boot Up Vision
#./bootUpVision.sh

#Reboot Vision
#num2=$(expr length "$(./../Viewers/checkML1.sh 2>/dev/null | egrep -m 1 "nanosec:")")

#More trouble than it's worth
#Checks if the otuput is "nanosec: 0"
# while [ "$num2" -eq 14 ]
# do
#     ./rebootVision.sh
#     ./bootUpVision.sh
# done
#./rebootVision.sh

#Testing reboot
#Check ML Conditional
#sleep 15
#num2=$(./../Viewers/checkML1.sh | egrep "nanosec: 0" | wc -l)

#./rebootVision.sh

#Verify if ML is broken
#num2=$(egrep "nanoseconds: 0.0" | wc -l)

#if [ 1 -eq 1 ]; then
#Wait until Pi is recognized
#while [ "$num1" -ne 1 ]; do; done