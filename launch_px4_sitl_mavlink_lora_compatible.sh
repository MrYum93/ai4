#!/bin/bash
##########################################
# TEMPLATE FOR USING SCRIPT TO START PX4 SITL SIMULATION
# AND BRIDGING VIRTUAL SERIAL PORTS PORTS
#
# Per Breum <pebre13@student.sdu.dk>
##########################################

# Set the home position for the simulation. If not set the home position
# is located in Zurich. This home position is located as the takeoff area
# in Odense Model airfield
export PX4_HOME_LAT=55.471979
export PX4_HOME_LON=10.414697

if [[ $1 = "" ]]; then
  echo "You did not supply the Firmware folder."
  exit
else
FIRMDIR=$1
fi

echo $FIRMDIR
# source ros - remeber to change to your version, e.g. melodic/kinetic
source /opt/ros/kinetic/setup.bash

# argument used to browse to your PX4 SITL firmware folder
cd $FIRMDIR

# Needed environment for running SITL
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

# # create 2 linked virtual serial ports
socat -d -d pty,raw,echo=0,link=/tmp/sim1 pty,raw,echo=0,link=/tmp/sim2 &

# # Bridge serial-udp
socat -d udp4-listen:14550 open:/tmp/sim1,raw,nonblock,waitlock=/tmp/s0.locak,echo=0,b115200,crnl &

# # launch basic PX4 SITL
roslaunch px4 rmuast_posix_sitl.launch 
