cmake_minimum_required(VERSION 3.0.2)
project(tuw_trinamic_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_python_setup()

include_directories()

catkin_install_python(PROGRAMS
     # scripts/myscript1
     # scripts/myscript2
     # nodes/mynode1
     # nodes/mynode2
     # src/tuw_trinamic_controller/motor_parameter_tuner.py
     src/tuw_trinamic_controller/motor_controller_node.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
