cmake_minimum_required(VERSION 3.5)

project(avionics_sim_test_runner C CXX)

# Find the Ignition-Math library
set(IGN_MATH_VER 4)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

# Find libXML2
find_package(LibXml2 REQUIRED)
message(STATUS LIBXML2_FOUND = ${LIBXML2_FOUND})
include_directories(${LIBXML2_INCLUDE_DIR})

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(CMAKE_USE_WIN32_THREADS_INIT 1)
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)

# Needed to include CodeCoverage.cmake
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/Jenkins)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# CMAKE does not handle relative paths, despite an (inoperative) flag, CMAKE_USE_RELATIVE_PATHS. 
# Use workaround below to make reference to files above. Cannot use to execute CMakes in those folders.
get_filename_component(DIR_ONE_ABOVE ../ ABSOLUTE)
get_filename_component(DIR_TWO_ABOVE ../../ ABSOLUTE)

add_subdirectory("googletest")

add_subdirectory("${DIR_ONE_ABOVE}/integration_tests" "${DIR_ONE_ABOVE}/integration_tests")
add_subdirectory("${DIR_ONE_ABOVE}/unit_tests" "${DIR_ONE_ABOVE}/unit_tests")

# Needed for code coverage tool to scan the source file
file(GLOB AVIONICS_SIM_SRCS
        "${DIR_TWO_ABOVE}/src/*.cpp"
        )

#Include a reference to find out where avionics_sim headers are.
include_directories(include ${DIR_ONE_ABOVE}/include ${DIR_ONE_ABOVE}/integration_tests ${DIR_TWO_ABOVE}/include)

add_executable(avionics_sim_test_runner
    src/IntegrationTestEnvironment.cpp
    src/main.cpp
    src/xmlparser.cpp
    src/utilities.cpp
    ${AVIONICS_SIM_SRCS}
    ${AVIONICS_SIM_INTEGRATION_TEST_SRC}
    ${AVIONICS_SIM_UNIT_TEST_SRC}
)

target_link_libraries(avionics_sim_test_runner
	gtest
	gmock
	#avionics_sim # No need for this anymore. Already building with the source classes.
	${LIBXML2_LIBRARIES}
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-g -O0 -s -Wall -fprofile-arcs -ftest-coverage")

    include(CodeCoverage)

    include(CodeStyle)
    
    SETUP_TARGET_FOR_COVERAGE_COBERTURA ( ${PROJECT_NAME}_coverage avionics_sim_test_runner output_coverage cppcheck)
endif()
