cmake_minimum_required(VERSION 3.0.2)
project(chatgpt_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  ChatPrompt.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# Install scripts

install(PROGRAMS
  script/chatgpt_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
