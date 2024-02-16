cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
    effort_controllers
    angles
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    hardware_interface
    pluginlib
    realtime_tools
    std_msgs
    urdf
    roscpp
    sensor_msgs
    roslint
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    hardware_interface
    realtime_tools
    std_msgs
    urdf
  LIBRARIES
    ${PROJECT_NAME}
  INCLUDE_DIRS
    include
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hero_chassis_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install plugins
install(FILES hero_chassis_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

