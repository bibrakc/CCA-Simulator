cmake_minimum_required(VERSION 3.26)
project(CCA-Simulator LANGUAGES CXX)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

# Add the executable for the action.cpp source file
add_subdirectory(Src)
