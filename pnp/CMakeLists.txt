cmake_minimum_required(VERSION 3.10)
project(pnp_solver)

find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(pnp_solver
    src/main.cpp
    src/armor_detector_base.cpp
    src/armor_detector.cpp
    src/armor_detector_channel.cpp
    src/pose_solver.cpp
)
target_link_libraries(pnp_solver ${OpenCV_LIBS})
