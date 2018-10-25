#!/bin/bash
##########################################
# Launch gcs node - terminal program to view drone information
#
# Per Breum <pebre13@student.sdu.dk>
##########################################

source devel/setup.bash

roslaunch gcs_terminal_node gcs.launch
