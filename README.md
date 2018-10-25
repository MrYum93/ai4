# RMUASD-Team3-2018

## Always remember to use catkin_make first for updated changes

## PX4 simulation, PX4-Mavlink_lora bridge and GCS terminal node is to run their own terminal window

#### To start simulation run (pass your PX4 Firmware folder as argument)

./launch_px4_sitl_mavlink_lora_compatible.sh /path_to_your_px4_firmware_folder

#### Run Mavlink Lora simulation with script

 ./simulation_mavlink_launch.sh

#### GCS terminal node

To launch GCS node - run following script in a new terminal window

./launch_gcs_terminal.sh

You can use keystrokes to control the drone
- Takeoff
- Arm
- Activate offboard
- The keys w,a,s,d will let you move drone around with a small dislacement
