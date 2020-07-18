PROJECT(PlotLib)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
CMAKE_POLICY(SET CMP0015 NEW)

file(GLOB SRC_FILES "*.cc")
add_library(PlotLib ${SRC_FILES})
target_include_directories(PlotLib PRIVATE ${PYTHON_INCLUDE_DIRS})
target_link_libraries(PlotLib CmnLib GnssLib InsLib ${PYTHON_LIBRARIES})