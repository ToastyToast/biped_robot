cmake_minimum_required(VERSION 2.8.3)
project(biped_description)

set(CMAKE_CXX_STANDARD 11)

set(${PROJECT_NAME}_CATKIN_DEPS
        joint_state_publisher
        robot_state_publisher
        rviz
        xacro
        )

find_package(catkin REQUIRED COMPONENTS
        ${${PROJECT_NAME}_CATKIN_DEPS}
        )

catkin_package(
        CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_DEPS}
)