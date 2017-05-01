project(ExtendedKF)

#cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/FusionKalmanFilter.cpp
    src/Measurement.cpp
    src/Math.cpp
    src/main.cpp)

add_executable(ExtendedKF ${sources})
