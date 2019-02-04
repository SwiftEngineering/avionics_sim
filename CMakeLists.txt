cmake_minimum_required(VERSION 3.5)

project(avionics_sim)

add_library(avionics_sim
	src/Battery_model.cpp
	src/Differential_pressure_sensor_model.cpp
	src/Exponential_smoothing_filter.cpp
	src/GaussianMarkov_noise.cpp
	src/NPolynomial_Eval.cpp
	src/US_1976_atmosphere.cpp
)

target_include_directories(avionics_sim PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(AVIONICS_SIM_HEADERS 
	include/avionics_sim/Battery_model.hpp
	include/avionics_sim/Differential_pressure_sensor_model.hpp
	include/avionics_sim/Exponential_smoothing_filter.hpp
	include/avionics_sim/GaussianMarkov_noise.hpp
	include/avionics_sim/Math_util.hpp
	include/avionics_sim/NPolynomial_Eval.hpp
	include/avionics_sim/US_1976_atmosphere.hpp
)

install(TARGETS avionics_sim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  COMPONENT avionics-sim
)

install(
  FILES ${AVIONICS_SIM_HEADERS} DESTINATION include/avionics_sim
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
