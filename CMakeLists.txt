cmake_minimum_required(VERSION 3.24)

project(airbrake_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(airbrake_core
    src/flight_sample.cpp
    src/atmosphere.cpp
    src/drag_table.cpp
    src/drag_table_csv.cpp
    src/vertical_dynamics.cpp
    src/apogee_predictor.cpp
    src/deployment_controller.cpp
)

target_include_directories(airbrake_core
    PUBLIC include
)

add_executable(airbrake_sim
    src/main.cpp
)

target_link_libraries(airbrake_sim
    PRIVATE airbrake_core
)
