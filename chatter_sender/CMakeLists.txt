cmake_minimum_required(VERSION 2.8.3)
project(chatter_sender)

find_package(catkin REQUIRED COMPONENTS chatter_msgs roscpp rostest)

generate_messages(DEPENDENCIES chatter_msgs)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(chatter_sender src/chatter_sender.cpp)

target_link_libraries(chatter_sender
   ${catkin_LIBRARIES}
)

add_subdirectory(test)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
