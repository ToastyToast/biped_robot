cmake_minimum_required(VERSION 2.8.3)
project(biped_rviz)

set(CMAKE_CXX_STANDARD 11)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp
        visualization_msgs

        biped_msgs
        biped_kinematics_dynamics
        biped_control
        )

find_package(catkin REQUIRED COMPONENTS
        ${${PROJECT_NAME}_CATKIN_DEPS}
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
        ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
        include/
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

set(${PROJECT_NAME}_LIBRARIES
        ${catkin_LIBRARIES}
        )

set(${PROJECT_NAME}_SRC
        include/biped_rviz/markers/basic_marker.h
        src/markers/basic_marker.cpp
        include/biped_rviz/plugins/plugin.h
        )


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_LIBRARIES})

add_executable(biped_marker_test_node src/nodes/biped_marker_test_node.cpp)
add_dependencies(biped_marker_test_node biped_msgs_generate_messages_cpp)
target_link_libraries(biped_marker_test_node ${PROJECT_NAME})
