cmake_minimum_required(VERSION 3.16)
project(simulink_mini CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

find_package(nlohmann_json 3.2.0 REQUIRED)

include_directories(include)

add_executable(test_scheduler tests/test_scheduler.cpp)
target_link_libraries(test_scheduler PRIVATE nlohmann_json::nlohmann_json)

add_executable(test_simulation tests/test_simulation.cpp)
target_link_libraries(test_simulation PRIVATE nlohmann_json::nlohmann_json)

add_executable(test_codegen tests/test_codegen.cpp)
target_link_libraries(test_codegen PRIVATE nlohmann_json::nlohmann_json)

add_executable(test_pid tests/test_pid.cpp)
target_link_libraries(test_pid PRIVATE nlohmann_json::nlohmann_json)

add_executable(test_codegen_pid tests/test_codegen_pid.cpp)
target_link_libraries(test_codegen_pid PRIVATE nlohmann_json::nlohmann_json)

add_executable(test_pso_tune tests/test_pso_tune.cpp)
target_link_libraries(test_pso_tune PRIVATE nlohmann_json::nlohmann_json)

# Copy test fixture JSON files next to the binary so relative paths work
# regardless of build directory.
file(COPY tests/valid_feedback_loop.json tests/broken_algebraic_loop.json tests/shm_oscillator.json
          tests/pid_step_response.json tests/broken_loop_through_pid.json tests/closed_loop_pid_plant.json
     DESTINATION ${CMAKE_BINARY_DIR}/tests)

# --- Python bindings (Week 4) ---
# Optional: only built if pybind11 is available, so a plain C++ checkout
# (no Python toolchain) still configures and builds the test suite fine.
find_package(Python3 COMPONENTS Interpreter Development QUIET)
if(Python3_FOUND)
    execute_process(
        COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
        OUTPUT_VARIABLE pybind11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYBIND11_CMAKEDIR_RESULT
    )
    if(PYBIND11_CMAKEDIR_RESULT EQUAL 0)
        list(APPEND CMAKE_PREFIX_PATH ${pybind11_CMAKE_DIR})
        find_package(pybind11 CONFIG QUIET)
    endif()
endif()

if(pybind11_FOUND)
    message(STATUS "pybind11 found -- building simmini_py Python extension module")
    pybind11_add_module(simmini_py python/bindings.cpp)
    target_link_libraries(simmini_py PRIVATE nlohmann_json::nlohmann_json)
else()
    message(STATUS "pybind11 not found -- skipping simmini_py Python extension module (C++ test suite still builds)")
endif()
