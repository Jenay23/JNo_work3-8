cmake_minimum_required(VERSION 3.10)
project(opencv_demo)

find_package(OpenCV REQUIRED)

add_executable(try_1 demo/try_1.cpp)
target_link_libraries(try_1 ${OpenCV_LIBS})

add_executable(try_2 demo/try_2.cpp)
target_link_libraries(try_2 ${OpenCV_LIBS})

add_executable(demo demo/demo.cpp)
target_link_libraries(demo ${OpenCV_LIBS})
