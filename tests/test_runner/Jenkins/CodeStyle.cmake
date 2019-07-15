# Written by I. Ukpo, 5-23-2019
#Adapted from https://arcanis.me/en/2015/10/17/cppcheck-and-clang-format
# additional target to perform cppcheck run, requires cppcheck

# get all project files
# HACK this workaround is required to avoid qml files checking ^_^
#file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)
#foreach (SOURCE_FILE ${ALL_SOURCE_FILES})
#    string(FIND ${SOURCE_FILE} ${PROJECT_TRDPARTY_DIR} PROJECT_TRDPARTY_DIR_FOUND)
#    if (NOT ${PROJECT_TRDPARTY_DIR_FOUND} EQUAL -1)
#        list(REMOVE_ITEM ALL_SOURCE_FILES ${SOURCE_FILE})
#    endif ()
#endforeach ()

#cppcheck --enable=all --std=c++11 --std=c99 --std=posix --xml --xml-version=2 "$WORKSPACE/avionics_sim_unit_tests" "$WORKSPACE/avionics_sim/src" -I "$WORKSPACE/avionics_sim/include/avionics_sim" -i "$WORKSPACE/googletest" 2> "$WORKSPACE/report_cppcheck.xml"

#${CMAKE_SOURCE_DIR}/avionics_sim_unit_tests
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
	-i "${CMAKE_SOURCE_DIR}/googletest"
	2> "${CMAKE_SOURCE_DIR}/report_cppcheck.xml"
        #${ALL_SOURCE_FILES}
)

