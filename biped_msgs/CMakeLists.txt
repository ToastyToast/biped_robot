cmake_minimum_required(VERSION 2.8.3)
project(biped_msgs)

set(CMAKE_CXX_STANDARD 11)

# Deps is for messages only here
set(${PROJECT_NAME}_CATKIN_DEPS
        std_msgs
        sensor_msgs
        geometry_msgs
        control_msgs
        trajectory_msgs
        )

find_package(catkin REQUIRED COMPONENTS
        message_generation
        ${${PROJECT_NAME}_CATKIN_DEPS}
        )

add_message_files(
        DIRECTORY msg
        FILES
)

add_service_files(
        DIRECTORY srv
        FILES
)

generate_messages(
        DEPENDENCIES
        ${${PROJECT_NAME}_CATKIN_DEPS}
)

catkin_package(
        CATKIN_DEPENDS
        message_runtime
        ${${PROJECT_NAME}_CATKIN_DEPS}
)