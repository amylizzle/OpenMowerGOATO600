cmake_minimum_required(VERSION 3.10)
project(ecovacs_bridge)

find_package(catkin REQUIRED COMPONENTS
    rospy
    open_mower
    mower_msgs
    ecovacs_messages
    sensor_msgs
    geometry_msgs
    std_msgs
    nmea_msgs
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS rospy open_mower mower_msgs ecovacs_messages sensor_msgs geometry_msgs std_msgs nmea_msgs
)

install(PROGRAMS
    scripts/bridge_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
