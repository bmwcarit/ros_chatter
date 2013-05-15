cmake_minimum_required(VERSION 2.8.3)
project(chatter_receiver)

find_package(catkin REQUIRED COMPONENTS chatter_msgs roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(chatter_receiver src/chatter_receiver_node.cpp src/Receiver.cpp)

add_dependencies(chatter_receiver chatter_msgs_gencpp)

target_link_libraries(chatter_receiver
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
