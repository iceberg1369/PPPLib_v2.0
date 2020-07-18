PROJECT(PPPLib_app)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
CMAKE_POLICY(SET CMP0015 NEW)

add_executable(TimeConver TimeConver.cc)
target_link_libraries(TimeConver PPPLib)

add_executable(CoordConver CoordConver.cc)
target_link_libraries(CoordConver PPPLib)
