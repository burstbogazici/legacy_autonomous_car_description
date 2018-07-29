cmake_minimum_required(VERSION 2.8.3)
project(autonomous_car_description)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    message_runtime
    genmsg
    sensor_msgs
    std_msgs
    geometry_msgs
    cv_bridge
    gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
    CATKIN_DEPENDS
    gazebo_ros
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(AutonomousCarPlugin SHARED src/AutonomousCarPlugin.cpp)
add_dependencies(AutonomousCarPlugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AutonomousCarPlugin
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


add_executable(AutonomousCarEncoder src/AutonomousCarEncoder.cpp)
add_dependencies(AutonomousCarEncoder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(AutonomousCarEncoder
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS AutonomousCarPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
