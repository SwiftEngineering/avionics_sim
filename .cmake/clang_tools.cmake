# This file defines custom targets and options to support running
# or enabling Clang tooling.

find_program(CLANG_TIDY NAMES clang-tidy clang-tidy-6.0)
if (CLANG_TIDY)
    message(STATUS "Source Files: ${Sources}")
    message(STATUS "Cmake Source Dir: ${CMAKE_SOURCE_DIR}")
    add_custom_target(
            clang-tidy
            COMMAND ${CLANG_TIDY}
            ${Sources}
            --
            -std=c++11
            -I ${CMAKE_SOURCE_DIR}/include
    )
endif ()
