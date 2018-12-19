
add_library(avionics_sim
	src/Battery_model.cpp
	src/Differential_pressure_sensor_model.cpp
	src/Exponential_smoothing_filter.cpp
	src/GaussianMarkov_noise.cpp
	src/Poly_fit_3rd_degree.cpp
	src/US_1976_atmosphere.cpp
)

target_include_directories(avionics_sim PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)
