add_executable(wanderer wander.cpp)
target_link_libraries(wanderer ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(wanderer ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS wanderer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)