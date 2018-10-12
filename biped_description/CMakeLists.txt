cmake_minimum_required(VERSION 2.8.3)
project(biped_description)

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
        DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)