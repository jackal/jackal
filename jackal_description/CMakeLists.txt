cmake_minimum_required(VERSION 3.0.2)
project(jackal_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/description.launch)
endif()
install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(WIN32)
install(PROGRAMS scripts/env_run.bat
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else()
install(PROGRAMS scripts/env_run
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif()
