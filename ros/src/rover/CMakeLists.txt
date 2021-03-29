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
        nodes/rover_manual_driver.py
        nodes/xbox_controller.py
        nodes/rover_diff_drive_controller.py
        nodes/xbox_topic.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()

