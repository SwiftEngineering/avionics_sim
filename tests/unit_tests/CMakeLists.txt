
SET(AVIONICS_SIM_UNIT_TEST_SRC
	${CMAKE_CURRENT_SOURCE_DIR}/Test_battery_model.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_diff_press.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_Exponential_smoothing_filter.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_GaussianMarkov_noise.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_Prop_wash.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_US_1976_atmosphere.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_Bilinear_interp.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_Lift_drag_model.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/Test_Coordinate_Utils.cpp
	PARENT_SCOPE
)
