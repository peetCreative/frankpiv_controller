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
    add_rostest(launch/frankpiv_controller_movements.test)

    catkin_install_python(
            PROGRAMS sim_movement_test.py
            DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif(BUILD_INTEGRATION_TESTS)