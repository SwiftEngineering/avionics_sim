cmake_minimum_required(VERSION 3.10)

# ---- Begin Adding Google Test Configuration --------
# Check for GTest. If not found, pull from googletest repo and use.
# More information can be found at https://github.com/google/googletest/blob/master/googletest/README.md
# find_package(GTest)
# if(NOT GTEST_LIBRARY)
#     message(" -- Package GTest not found, attempting fetching from source...")
#     include(FetchContent)

#     FetchContent_Declare(
#         googletest
#         GIT_REPOSITORY https://github.com/google/googletest.git
#         GIT_TAG        release-1.10.0
#     )

#     FetchContent_GetProperties(googletest)
#     if(NOT googletest_POPULATED)
#         FetchContent_Populate(googletest)
#         add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
#         message(" -- Successfully fetched GTest from source!")
#     else()
#         message(" -- Unable to fetch GTestfrom source, check internet connection")
#     endif()
# else()
#    message(STATUS " -- GTest found. Please ensure you have GTest version 1.7 or later.")
# endif()
#add_subdirectory(googletest)
#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
find_package(GTest)

# ---- End Adding Google Test Configuration ----------

# A shared directory for the xml output of each test
SET(TEST_OUTPUT_XML_DIR ${PROJECT_BINARY_DIR}/publish/tests/xml/)
add_subdirectory(unit)
# add_subdirectory(integration)
# add_subdirectory(system)

# ALL TESTS FOR PROJECT
# add_executable(${TEST_PROJECT_NAME} ${SOURCES} )

# target_link_libraries(${TEST_PROJECT_NAME} PUBLIC
#     gtest_main
#     gmock_main
#     ${PROJECT_NAME}
# )
# add_test(
#     NAME ${TEST_PROJECT_NAME}
#     COMMAND ${TEST_PROJECT_NAME}
#     CONFIGURATIONS UNIT
# )
