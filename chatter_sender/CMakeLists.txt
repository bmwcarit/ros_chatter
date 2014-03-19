cmake_minimum_required(VERSION 2.8.3)
project(chatter_sender)

find_package(catkin REQUIRED COMPONENTS chatter_msgs roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(chatter_sender src/chatter_sender.cpp)

add_dependencies(chatter_sender chatter_msgs_gencpp)

target_link_libraries(chatter_sender
   ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_subdirectory(test)
endif()

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
