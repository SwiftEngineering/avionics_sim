file(GLOB_RECURSE UNIT_TEST_SRC  "*.cpp")

list(LENGTH UNIT_TEST_SRC number_of_srcs)

# UNIT TESTS: Add the unit tests if the sources exist
if (number_of_srcs EQUAL 0)
    message(WARNING " No source files for unit test")
else()
    add_executable(unit_tests_exe ${UNIT_TEST_SRC})
    target_link_libraries(unit_tests_exe PUBLIC
    GTest::GTest GTest::Main
        # gtest_main
        # gmock
        # ${LIBXML2_LIBRARIES}
    	# ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
	    # ${Boost_FILESYSTEM_LIBRARY}
    	# ${Boost_LIBRARIES}
        ${PROJECT_NAME}
    )
    # TODO: consider using cmake's gtest_discover_tests
    add_test(
        NAME unit_tests
        COMMAND unit_tests_exe --gtest_output=xml:${TEST_OUTPUT_XML_DIR} --gtest_catch_exceptions=true
    )
endif()
