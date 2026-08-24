cmake_minimum_required(VERSION 3.24)

project(airbrake_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.8.1
)

FetchContent_MakeAvailable(Catch2)

add_library(airbrake_core
    src/flight_sample.cpp
    src/atmosphere.cpp
    src/drag_table.cpp
    src/drag_table_csv.cpp
)

target_include_directories(airbrake_core
    PUBLIC include
)

add_executable(airbrake_sim_tests
    tests/smoke_test.cpp
    tests/flight_sample_test.cpp
)

add_executable(airbrake_sim
    src/main.cpp
)

target_link_libraries(airbrake_sim
    PRIVATE airbrake_core
)

target_link_libraries(airbrake_sim_tests
    PRIVATE Catch2::Catch2WithMain
    airbrake_core
)

include(Catch)
catch_discover_tests(airbrake_sim_tests)