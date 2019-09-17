cmake_minimum_required(VERSION 2.8.3)
project(object_follower)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        message_generation
        roscpp
        gazebo_ros
        rospy
        std_msgs
        )

#add_message_files(
#  FILES
#)

#generate_messages(
#        DEPENDENCIES
#        std_msgs  # Or other packages containing msgs
#)

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs gazebo_ros
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS})