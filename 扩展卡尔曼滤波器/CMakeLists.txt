cmake_minimum_required(VERSION 3.10)
project(work6)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_executable(work6
    work6.cpp
    extended_kalman_filter/extended_kalman_filter.cpp
)

add_executable(yaw_1d_ekf
    yaw_1d_ekf.cpp
    extended_kalman_filter/extended_kalman_filter.cpp
)

target_include_directories(work6 PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(work6 Eigen3::Eigen)

target_include_directories(yaw_1d_ekf PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(yaw_1d_ekf Eigen3::Eigen)
