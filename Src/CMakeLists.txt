cmake_minimum_required(VERSION 3.26)

set(executable_name CCASimulator)
project(${executable_name} LANGUAGES CXX)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
add_definitions(-DDEBUG_CODE=${DEBUG_CODE})

if(APPLE)
    set(CMAKE_C_COMPILER clang)
    set(CMAKE_CXX_COMPILER clang++)

    if(CMAKE_C_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_C_FLAGS "-Xpreprocessor -fopenmp")
        set(OpenMP_C_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY omp)
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang\$")
        set(OpenMP_CXX_FLAGS "-Xpreprocessor -fopenmp")
        set(OpenMP_CXX_LIB_NAMES "omp")
        set(OpenMP_omp_LIBRARY omp)
    endif()

endif()

find_package(OpenMP REQUIRED)

# Add the executable for the action.cpp source file
add_executable(${executable_name} action.cpp)

set(includeDirs "/opt/homebrew/opt/libomp/include")
target_include_directories(${executable_name} PRIVATE ${includeDirs})

set_source_files_properties(action.cpp PROPERTIES LANGUAGE CXX)

target_link_libraries(${executable_name} PRIVATE OpenMP::OpenMP_CXX)

