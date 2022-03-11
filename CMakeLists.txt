cmake_minimum_required(VERSION 3.0.2)
project(frankpiv_controller)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
        gazebo_dev
        controller_interface
        dynamic_reconfigure
        eigen_conversions
        franka_hw
        franka_gripper
        franka_gazebo
        gazebo_ros_control
        geometry_msgs
        hardware_interface
        tf
        tf_conversions
        message_generation
        pluginlib
        realtime_tools
        roscpp
        rospy
        pivot_control_messages_ros
        )

find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/compliance_param.cfg
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES frankpiv_controller
        CATKIN_DEPENDS
        controller_interface
        dynamic_reconfigure
        eigen_conversions
        franka_hw
        franka_gripper
        franka_gazebo
        gazebo_ros_control
        geometry_msgs
        hardware_interface
        tf
        tf_conversions
        message_runtime
        pluginlib
        realtime_tools
        roscpp
        pivot_control_messages_ros
        INCLUDE_DIRS include
        LIBRARIES franka_hw_sim
        DEPENDS Franka
)

add_library(frankpiv_controller
        src/pivot_controller.cpp
        )

add_dependencies(frankpiv_controller
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        pivot_control_messages_ros_generate_messages_cpp
#        ${PROJECT_NAME}_gencpp
        ${PROJECT_NAME}_gencfg
        )

target_link_libraries(frankpiv_controller PUBLIC
        ${Franka_LIBRARIES}
        ${catkin_LIBRARIES}
        )

target_include_directories(frankpiv_controller SYSTEM PUBLIC
        ${Franka_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        )
target_include_directories(frankpiv_controller PUBLIC
        include
        )


## Installation
install(TARGETS frankpiv_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
install(FILES frankpiv_controller_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
catkin_install_python(
        PROGRAMS scripts/trajectory_generator.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Tools
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ClangTools.cmake OPTIONAL
        RESULT_VARIABLE CLANG_TOOLS
        )
if(CLANG_TOOLS)
    file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
    file(GLOB_RECURSE HEADERS
            ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
            )
    add_format_target(frankpiv_controller FILES ${SOURCES} ${HEADERS})
    add_tidy_target(frankpiv_controller
            FILES ${SOURCES}
            DEPENDS frankpiv_controller
            )
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/PepTools.cmake OPTIONAL
        RESULT_VARIABLE PEP_TOOLS
        )
if(PEP_TOOLS)
    file(GLOB_RECURSE PYSOURCES ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py)
    add_pyformat_target(franka_control FILES ${PYSOURCES})
endif()

if(CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()