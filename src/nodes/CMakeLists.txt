add_executable(push_com_action push_com_action.cpp)
target_link_libraries(push_com_action ${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS push_com_action
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
