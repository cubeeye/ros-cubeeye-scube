cmake_minimum_required(VERSION 2.8.3)
project(cubeeye_scube)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

 generate_dynamic_reconfigure_options(
   cfg/scube_depth.cfg
 )


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include/CubeEye
)

add_executable(${PROJECT_NAME}_node src/scubeNode.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libCubeEye.so
)

