cmake_minimum_required(VERSION 3.1.0)
project(stage_ros)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(stage REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS}
)

catkin_package()

set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()

set (CMAKE_CXX_STANDARD 11)

add_library(simbase src/robot.cpp src/stageros.cpp)

target_link_libraries(simbase
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${STAGE_CORE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)

# Declare a cpp executable
add_executable(stageros src/main.cpp)
target_link_libraries(stageros simbase)

if(STAGE_USE_GUI)
	add_definitions(-DHAS_STAGE_GUI)
	add_executable(stagerosg src/main.cpp)
	target_link_libraries(stagerosg simbase ${STAGE_LIBRARIES})
endif(STAGE_USE_GUI)

target_link_libraries(stageros
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(stageros ${catkin_EXPORTED_TARGETS})
endif()

add_subdirectory(src/behaviours)
## Install

install(PROGRAMS scripts/upgrade-world.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS stageros simbase
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/hztest.xml)
  add_rostest(test/intensity_test.xml)
endif()
