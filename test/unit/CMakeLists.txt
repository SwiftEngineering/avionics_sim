file(GLOB_RECURSE UNIT_TEST_SRC  "*.cpp")

list(LENGTH UNIT_TEST_SRC number_of_srcs)

# UNIT TESTS: Add the unit tests if the sources exist
if (number_of_srcs EQUAL 0)
    message(WARNING " No source files for unit test")
else()
    add_executable(unit_tests_exe ${UNIT_TEST_SRC})
    target_link_libraries(unit_tests_exe PUBLIC
    GTest::GTest GTest::Main
        ${PROJECT_NAME}
    )
    gtest_discover_tests(unit_tests_exe
        EXTRA_ARGS --gtest_output=xml:${TEST_OUTPUT_XML_DIR} --gtest_catch_exceptions=true
    )
endif()
