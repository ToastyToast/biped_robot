cmake_minimum_required(VERSION 2.8.3)
project(biped_kinematics_dynamics)

set(CMAKE_CXX_STANDARD 11)

set(${PROJECT_NAME}_CATKIN_DEPS
        roscpp

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
)

set(${PROJECT_NAME}_LIBRARIES
        ${catkin_LIBRARIES}
        )

set(${PROJECT_NAME}_SRC
        include/biped_kinematics_dynamics/link_info.h
        src/link_info.cpp
        src/nodes/kinematics_node.cpp
        )


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRC})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_LIBRARIES})

add_executable(kinematics_node src/nodes/kinematics_node.cpp)
target_link_libraries(kinematics_node ${PROJECT_NAME})
