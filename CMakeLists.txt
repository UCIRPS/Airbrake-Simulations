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

add_executable(airbrake_sim_tests
    tests/smoke_test.cpp
)

target_link_libraries(airbrake_sim_tests
    PRIVATE Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(airbrake_sim_tests)