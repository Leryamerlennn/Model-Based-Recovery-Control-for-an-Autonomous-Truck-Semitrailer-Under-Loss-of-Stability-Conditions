cmake_minimum_required(VERSION 3.16)

project(truck_dynamics VERSION 0.1.0 LANGUAGES CXX)

option(TRUCK_MODEL_BUILD_TESTS "Build truck model tests" ON)
option(TRUCK_MODEL_BUILD_EXAMPLES "Build truck model examples" ON)

add_library(truck_model
    src/truck_model.cpp
    src/mppi_controller.cpp
)
add_library(truck_model::truck_model ALIAS truck_model)

target_include_directories(truck_model
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(truck_model PUBLIC cxx_std_17)
target_compile_options(truck_model PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall;-Wextra;-Wpedantic>
)

if(TRUCK_MODEL_BUILD_EXAMPLES)
    add_executable(simulate_truck examples/simulate.cpp)
    target_link_libraries(simulate_truck PRIVATE truck_model::truck_model)
endif()

if(TRUCK_MODEL_BUILD_TESTS)
    enable_testing()
    add_executable(truck_model_tests tests/truck_model_tests.cpp)
    target_link_libraries(truck_model_tests PRIVATE truck_model::truck_model)
    add_test(NAME truck_model_tests COMMAND truck_model_tests)
endif()

