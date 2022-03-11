find_package(rostest REQUIRED)

add_rostest_gtest(frankpiv_controller_test
        launch/frankpiv_controller_test.test
        main.cpp
        )

add_dependencies(frankpiv_controller_test
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(frankpiv_controller_test
        ${catkin_LIBRARIES}
        )

target_include_directories(frankpiv_controller_test PUBLIC
        ${catkin_INCLUDE_DIRS}
        )

add_rostest_gtest(frankpiv_controller_movements
        launch/frankpiv_controller_movements.test
        frankpiv_controller_movements.cpp
        )

add_dependencies(frankpiv_controller_movements
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(frankpiv_controller_movements
        ${catkin_LIBRARIES}
        )

target_include_directories(frankpiv_controller_movements PUBLIC
        ${catkin_INCLUDE_DIRS}
        )
