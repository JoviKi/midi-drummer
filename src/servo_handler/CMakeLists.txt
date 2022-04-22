cmake_minimum_required(VERSION 3.0.2)
project(drum_client)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_arduino
  rosserial_client
  std_msgs
  message_generation
)

add_message_files(
  FILES
  wmsg.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   spos.srv
   )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  CATKIN_DEPENDS message_runtime
)


