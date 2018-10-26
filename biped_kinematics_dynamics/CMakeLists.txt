cmake_minimum_required(VERSION 2.8.3)
project(biped_kinematics_dynamics)

set(CMAKE_CXX_STANDARD 11)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp
        tf2
        tf2_ros
        urdf

        biped_msgs
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
        include/biped_kinematics_dynamics/math/lie_groups.h
        include/biped_kinematics_dynamics/robot_tree/robot_tree.h
        src/robot_tree/robot_tree.cpp
        include/biped_kinematics_dynamics/robot_tree/robot_link.h
        src/robot_tree/robot_link.cpp
        include/biped_kinematics_dynamics/robot_tree/robot_joint.h
        src/robot_tree/robot_joint.cpp
        include/biped_kinematics_dynamics/robot_tree_publisher.h
        src/robot_tree_publisher.cpp
        include/biped_kinematics_dynamics/robot_tree/solvers/robot_tree_ik_solver_analytical.h
        src/robot_tree/solvers/robot_tree_ik_solver_analytical.cpp
        include/biped_kinematics_dynamics/robot_tree/solvers/robot_tree_ik_solver_numerical.h
        src/robot_tree/solvers/robot_tree_ik_solver_numerical.cpp)


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_LIBRARIES})

add_executable(biped_kinematics_node src/nodes/biped_kinematics_node.cpp)
add_dependencies(biped_kinematics_node biped_msgs_generate_messages_cpp)
target_link_libraries(biped_kinematics_node ${PROJECT_NAME})
