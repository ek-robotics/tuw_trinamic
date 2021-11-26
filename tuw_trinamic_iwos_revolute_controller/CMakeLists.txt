cmake_minimum_required(VERSION 3.0.2)
project(tuw_trinamic_iwos_revolute_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

include_directories()

catkin_install_python(PROGRAMS
  src/tuw_trinamic_iwos_revolute_controller/revolute_controller_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
