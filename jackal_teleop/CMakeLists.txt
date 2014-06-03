cmake_minimum_required(VERSION 2.8.3)
project(jackal_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

add_executable(${PROJECT_NAME}_simple_joy_node src/simple_joy_node.cpp)
target_link_libraries(${PROJECT_NAME}_simple_joy_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_simple_joy_node
                      PROPERTIES OUTPUT_NAME simple_joy_node
                      PREFIX "")

install(TARGETS ${PROJECT_NAME}_simple_joy_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}) 

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}) 
