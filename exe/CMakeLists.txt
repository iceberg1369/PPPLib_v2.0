PROJECT(test_main)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
CMAKE_POLICY(SET CMP0015 NEW)

add_subdirectory(app)

add_executable(test_main test.cc)
target_link_libraries(test_main PPPLib)

add_executable(SPP SPP.cc)
target_link_libraries(SPP PPPLib)

add_executable(PPP PPP.cc)
target_link_libraries(PPP PPPLib)

add_executable(PPK PPK.cc)
target_link_libraries(PPK PPPLib)

add_executable(FUSION FUSION.cc)
target_link_libraries(FUSION PPPLib)