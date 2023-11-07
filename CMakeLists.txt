cmake_minimum_required(VERSION 3.26)
project(simplified_four_wheel_model)

set(CMAKE_CXX_STANDARD 23)

add_executable(simplified_four_wheel_model main.cpp
        vehicle_model_fw_simplified.cpp
        vehicle_model_fw_simplified.h
        vehicle_simulator.cpp
        vehicle_simulator.h
        inputs.cpp
        inputs.h)
