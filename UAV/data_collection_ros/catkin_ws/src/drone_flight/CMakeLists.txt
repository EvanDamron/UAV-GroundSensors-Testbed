cmake_minimum_required(VERSION 2.8.3)
project(drone_flight)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp mavros
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(flight_execution src/flight_execution.cpp)
target_link_libraries(flight_execution ${catkin_LIBRARIES})
add_dependencies(flight_execution ${catkin_EXPORTED_TARGETS})

add_executable(manager src/manager.cpp)
target_link_libraries(manager ${catkin_LIBRARIES})
add_dependencies(manager ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark executables and/or libraries for installation
#install(TARGETS
#  ${PROJECT_NAME}_node
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
