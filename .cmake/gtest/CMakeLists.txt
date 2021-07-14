# ---- Begin Adding Google Test Configuration --------
# Check for GTest. If not found, pull from googletest repo and use.
# More information can be found at https://github.com/google/googletest/blob/master/googletest/README.md
find_package(GTest)
if(NOT GTEST_LIBRARY)
    message(" -- Package GTest not found, attempting fetching from source...")
    include(FetchContent)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.10.0
    )

    FetchContent_MakeAvailable(googletest)
    set(GTEST_LIBS gtest gtest_main gmock)

    message(" -- Successfully fetched GTest from source!")
else()
    set(GTEST_LIBS GTest::GTest GTest::Main)
    message(STATUS " -- GTest found. Please ensure you have GTest version 1.7 or later.")
endif()