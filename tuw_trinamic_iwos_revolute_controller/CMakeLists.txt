cmake_minimum_required(VERSION 3.0.2)
project(tuw_trinamic_iwos_revolute_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roslint
  dynamic_reconfigure
)

roslint_python()

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/reconfigure_node.cfg
  cfg/reconfigure_revolute.cfg
)

catkin_package()

include_directories()

catkin_install_python(PROGRAMS
  src/tuw_trinamic_iwos_revolute_controller/revolute_controller_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
