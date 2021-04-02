cmake_minimum_required(VERSION 3.0.2)
project(tuw_trinamic_controller)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(PROGRAMS
     # scripts/myscript1
     # scripts/myscript2
     # nodes/mynode1
     # nodes/mynode2
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
