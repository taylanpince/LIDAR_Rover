cmake_minimum_required(VERSION 3.0.2)
project(rover)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
)

catkin_package(
    
)

include_directories(

)

catkin_install_python(
    PROGRAMS
        nodes/rover_controller.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()

