#!/bin/bash

#Script test by DanieleDifra

cd ~/ROS_PO

if  [[ $1 = "-s" ]]; then
    echo "Option -s turned on"
    roslaunch edo_description display.launch
elif [[ $1 = "-k" ]]; then
    echo "Option -k turned on"
    roslaunch edo_description kinematic_simulation.launch
elif [[ $1 = "-d" ]]; then
    echo "Option -d turned on"
    roslaunch edo_description dynamic_simulation.launch
elif [[ $1 = "--help" ]]; then
    echo -e "Script version 0.0.1 \nUsage: ./test.sh [OPTION]\n-s,    Runs the display simulation\n-k,    Runs the kinematic simulation\n-d,    Runs the dynamic simulation "
else
    echo "You did not use any option, use --help to see manual"
fi


