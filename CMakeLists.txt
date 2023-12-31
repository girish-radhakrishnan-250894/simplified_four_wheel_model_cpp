cmake_minimum_required(VERSION 3.26)
project(simplified_four_wheel_model)

set(CMAKE_CXX_STANDARD 23)

include_directories(include)

set(SRC_FILES
    vehicle_model_fw_simplified.cpp
    vehicle_simulator.cpp
    inputs.cpp
    )

set(HEADER_FILES
    vehicle_model_fw_simplified.h
    vehicle_simulator.h
    inputs.h
)

add_library(simplified_four_wheel_model
        ${SRC_FILES}
        ${HEADER_FILES}
)

add_executable(simplified_four_wheel_model_executable
        main.cpp
)

target_link_libraries(simplified_four_wheel_model_executable PUBLIC simplified_four_wheel_model)

target_include_directories(simplified_four_wheel_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

########################################################################################################################
########################################################################################################################
# CMAKE CODE TO INCLUDE SCIPLOT FOR PLOTTING #
########################################################################################################################
########################################################################################################################
include(FetchContent) # If not included already

FetchContent_Declare(sciplot_content
        GIT_REPOSITORY https://github.com/sciplot/sciplot.git
        GIT_TAG master)

FetchContent_GetProperties(sciplot_content)
if(NOT sciplot_content_POPULATED)
    FetchContent_Populate(sciplot_content)
endif()

include_directories(${sciplot_content_SOURCE_DIR})

