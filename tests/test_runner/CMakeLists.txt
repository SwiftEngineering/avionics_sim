cmake_minimum_required(VERSION 3.5)

project(avionics_sim_test_runner C CXX)

#Check for GTest. If not found, pull from googletest repo and use.
#More information can be found at https://github.com/google/googletest/blob/master/googletest/README.md
find_package(GTest)
if(NOT GTEST_LIBRARY)
   message("GTest not found. Downloading GTest for this project...")
   #Instructions derived from https://github.com/google/googletest/blob/master/googletest/README.md
   configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
   execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
   RESULT_VARIABLE result
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
   if(result)
     message(FATAL_ERROR "CMake step for googletest failed: ${result}")
   endif()
   execute_process(COMMAND ${CMAKE_COMMAND} --build .
   RESULT_VARIABLE result
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
   if(result)
     message(FATAL_ERROR "Build step for googletest failed: ${result}")
   endif()

   # Prevent overriding the parent project's compiler/linker
   # settings on Windows
   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

   # The gtest/gtest_main targets carry header search path
   # dependencies automatically when using CMake 2.8.11 or
   # later. Otherwise we have to add them here ourselves.
   if (CMAKE_VERSION VERSION_LESS 2.8.11)
     include_directories("${gtest_SOURCE_DIR}/include")
   endif()
else()
   message(STATUS "GTest found. Please ensure you have GTest version 1.7 or later.")
endif()


# Find Boost
find_package(Boost REQUIRED COMPONENTS system filesystem)

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

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

add_subdirectory("${DIR_ONE_ABOVE}/integration_tests" "${DIR_ONE_ABOVE}/integration_tests")
add_subdirectory("${DIR_ONE_ABOVE}/unit_tests" "${DIR_ONE_ABOVE}/unit_tests")

# Needed for code coverage tool to scan the source file
file(GLOB AVIONICS_SIM_SRCS
        "${DIR_TWO_ABOVE}/src/*.cpp"
        )

file(GLOB AVIONICS_SIM_TEST_RUNNER_SRCS
        "src/*.cpp"
        )

#Include a reference to find out where avionics_sim headers are.
include_directories(include ${DIR_ONE_ABOVE}/include ${DIR_ONE_ABOVE}/integration_tests ${DIR_ONE_ABOVE}/unit_tests ${DIR_TWO_ABOVE}/include)

add_executable(avionics_sim_test_runner
    ${AVIONICS_SIM_TEST_RUNNER_SRCS}
    ${AVIONICS_SIM_SRCS}
    ${AVIONICS_SIM_INTEGRATION_TEST_SRC}
    ${AVIONICS_SIM_UNIT_TEST_SRC}
)

target_link_libraries(avionics_sim_test_runner
	-lpthread
	gtest_main
	gmock
	${LIBXML2_LIBRARIES}
    	ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
	${Boost_FILESYSTEM_LIBRARY}
    	${Boost_LIBRARIES}
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-g -O0 -s -Wall -fprofile-arcs -ftest-coverage")

    include(CodeCoverage)

    include(CodeStyle)
    
    SETUP_TARGET_FOR_COVERAGE_COBERTURA ( ${PROJECT_NAME}_coverage avionics_sim_test_runner output_coverage cppcheck)
endif()
