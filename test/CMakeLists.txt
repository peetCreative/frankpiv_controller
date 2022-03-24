find_package(GTest REQUIRED)
find_package(rostest REQUIRED)

if(BUILD_UNIT_TESTS)
    catkin_add_gtest(
            pivot_controller_unit_tests
            main.cpp
    )
    if(TARGET pivot_controller_unit_tests)
        target_link_libraries(
                pivot_controller_unit_tests
                ${catkin_LIBRARIES}
                ${PROJECT_NAME}
        )
    endif()
endif(BUILD_UNIT_TESTS)

## performance/integration test
if(BUILD_INTEGRATION_TESTS)
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
endif(BUILD_INTEGRATION_TESTS)