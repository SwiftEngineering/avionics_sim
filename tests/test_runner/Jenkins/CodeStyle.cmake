# Written by I. Ukpo, 5-23-2019
#Adapted from https://arcanis.me/en/2015/10/17/cppcheck-and-clang-format
# additional target to perform cppcheck run, requires cppcheck

add_custom_target(
        cppcheck
        COMMAND cppcheck
        --enable=all
        --std=c++11
		--std=c99
		--std=posix
		--xml
		--xml-version=2
        --template="[{severity}][{id}] {message} {callstack} \(On {file}:{line}\)"
		"${CMAKE_SOURCE_DIR}/../unit_tests"
		"${CMAKE_SOURCE_DIR}/../integration_tests"
		"${CMAKE_SOURCE_DIR}/../system_tests"
		"${CMAKE_SOURCE_DIR}/../../src"
		-I "${CMAKE_SOURCE_DIR}/../../include/avionics_sim"
		2> "${CMAKE_SOURCE_DIR}/report_cppcheck.xml"
			#${ALL_SOURCE_FILES}
)

