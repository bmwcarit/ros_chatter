add_executable(${PROJECT_NAME}_test EXCLUDE_FROM_ALL chatter_sender_test.cpp)

if(TARGET ${PROJECT_NAME}_test)
  
  target_link_libraries(${PROJECT_NAME}_test
    pthread rt boost_thread ${GTEST_LIBRARIES} ${roscpp_LIBRARIES})

  add_rostest(chatter_sender_test.launch)
  add_dependencies(tests ${PROJECT_NAME}_test)
  add_dependencies(${PROJECT_NAME}_test ${PROJECT_NAME})

endif()
