cmake_minimum_required(VERSION 2.8.3)
project(mavlink_lora)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  mavlink_lora_msg.msg
  mavlink_lora_status.msg
  mavlink_lora_pos.msg
  mavlink_lora_attitude.msg
  mavlink_lora_modes.msg
  mavlink_lora_local_setpoint.msg
  mavlink_lora_local_ned_pos.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mavlink_lora_node src/mavlink_lora_node.cpp src/mavlink_lora_lib.c src/serial.c)

# needed because the message file needs to be maked before the node using it
add_dependencies(mavlink_lora_node mavlink_lora_generate_messages_cpp)

target_link_libraries(mavlink_lora_node
  ${catkin_LIBRARIES}
)

