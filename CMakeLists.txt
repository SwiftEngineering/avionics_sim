cmake_minimum_required(VERSION 3.10)

project(avionics_sim C CXX)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-checks=*")

# Find the Boost Library
set(BOOST_VER 1.65.1)
find_package(Boost ${BOOST_VER} REQUIRED)

# Find the Ignition-Math library
set(IGN_MATH_VER 4)
find_package(ignition-math${IGN_MATH_VER} REQUIRED)

include_directories(include)
file(GLOB Sources "src/*.cpp")
file(GLOB Headers "include/*.hpp")

add_library(${PROJECT_NAME} ${Sources} ${Headers})

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME} ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER})

enable_testing()
add_subdirectory(test)

install(
  TARGETS avionics_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  COMPONENT avionics-sim
)

install(
  FILES ${Headers}
  DESTINATION include/avionics_sim
  COMPONENT avionics-sim-dev
)

SET(MAJOR_VERSION "0")
SET(MINOR_VERSION "1")
SET(PATCH_VERSION "0")

SET(CPACK_PACKAGE_VERSION_MAJOR ${MAJOR_VERSION})
SET(CPACK_PACKAGE_VERSION_MINOR ${MINOR_VERSION})
SET(CPACK_PACKAGE_VERSION_PATCH ${PATCH_VERSION})
set(CPACK_PACKAGE_CONTACT software@swiftengineering.com)

set(DEBIAN_PACKAGE_DEPENDS "")

set(CPACK_DEBIAN_PACKAGE_DEPENDS ${DEBIAN_PACKAGE_DEPENDS})
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

set(CPACK_COMPONENT_AVIONICS_SIM_DESCRIPTION "avionics_sim library")
set(CPACK_COMPONENT_AVIONICS_SIM_DEV_DESCRIPTION "avionics_sim library headers")

include(CPack)

cpack_add_component(avionics-sim
	DISPLAY_NAME avionics-sim
	DESCRIPTION "Libs for avionics sim"
	REQUIRED
)
cpack_add_component(avionics-sim-dev
	DISPLAY_NAME avionics-sim-dev
	DESCRIPTION "Dev headers for avionics sim"
	DEPENDS avionics-sim
)
