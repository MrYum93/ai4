#!/bin/sh

if [ "$1" = '' ]
then
echo failed to provide arg
echo "1: arduPilot"
echo "2: ros connect"
exit
fi

if [ "$1" = 1 ]
then
sim_vehicle.py -v ArduPlane --console --map
echo "plane launched"
fi

if [ "$1" = 2 ]
then
roslaunch apm.launch
echo "iris in gazebo launched"
fi
