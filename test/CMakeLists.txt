cmake_minimum_required(VERSION 3.10)

include(${PROJECT_SOURCE_DIR}/.cmake/gtest/CMakeLists.txt)

# ---- End Adding Google Test Configuration ----------

# A shared directory for the xml output of each test
SET(TEST_OUTPUT_XML_DIR ${PROJECT_BINARY_DIR}/publish/tests/xml/)
add_subdirectory(unit)
