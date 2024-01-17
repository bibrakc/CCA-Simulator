# BSD 3-Clause License
# 
# Copyright (c) 2023, Bibrak Qamar
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.26)

set(executable_name PageRank_Nested_Fixed_Iterations_CCASimulator)
project(${executable_name} LANGUAGES CXX)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)

if(NOT DEFINED THROTTLE)
    set(THROTTLE "true")
endif()
add_definitions(-DTHROTTLE=${THROTTLE})

if(NOT DEFINED RECVBUFFSIZE)
    set(RECVBUFFSIZE 1)
endif()
add_definitions(-DRECVBUFFSIZE=${RECVBUFFSIZE})

if(NOT DEFINED ANIMATION)
    set(ANIMATION "true")
endif()
add_definitions(-DANIMATION=${ANIMATION})

if(NOT DEFINED ACTIVE_PERCENT)
    set(ACTIVE_PERCENT "true")
endif()
add_definitions(-DACTIVE_PERCENT=${ACTIVE_PERCENT})

if(NOT DEFINED VICINITY)
    set(VICINITY 3)
endif()
add_definitions(-DVICINITY=${VICINITY})

if(NOT DEFINED MAXEDGESPERVERTEX)
    set(MAXEDGESPERVERTEX 30)
endif()
add_definitions(-DMAXEDGESPERVERTEX=${MAXEDGESPERVERTEX})

if(NOT DEFINED TERMINATION)
    set(TERMINATION "true")
endif()
add_definitions(-DTERMINATION=${TERMINATION})

if(NOT DEFINED THROTTLE_CONGESTION_THRESHOLD)
    set(THROTTLE_CONGESTION_THRESHOLD 181)
endif()
add_definitions(-DTHROTTLE_CONGESTION_THRESHOLD=${THROTTLE_CONGESTION_THRESHOLD})

if(NOT DEFINED NESTEDITERATIONS)
    set(NESTEDITERATIONS 10)
endif()
add_definitions(-DNESTEDITERATIONS=${NESTEDITERATIONS})


if(NOT DEFINED VERTEX_TYPE)
    set(VERTEX_TYPE "RecursiveParallelVertex")
endif()
add_definitions(-DVERTEX_TYPE=${VERTEX_TYPE})

find_package(OpenMP REQUIRED)

set(SourceDirectory ${CMAKE_CURRENT_SOURCE_DIR}/../../Source)
set(IncludeDirectory ${CMAKE_CURRENT_SOURCE_DIR}/../../Include)
set(UtilitiesDirectory ${CMAKE_CURRENT_SOURCE_DIR}/../../Utilities)

add_library(CCACommons
            ${SourceDirectory}/Operon.cpp
            ${SourceDirectory}/types.cpp       
            )

target_include_directories(CCACommons PUBLIC 
            ${IncludeDirectory}
            ${IncludeDirectory}/Datastructure)

target_compile_options(CCACommons PRIVATE -Werror -Wall -Wextra)

add_library(HtreeNework
            ${SourceDirectory}/HtreeNode.cpp
            ${SourceDirectory}/HtreeNetwork.cpp
            )

target_include_directories(HtreeNework PUBLIC 
            ${IncludeDirectory}
            )

target_compile_options(HtreeNework PRIVATE -Werror -Wall -Wextra)

target_link_libraries(HtreeNework PUBLIC CCACommons)

add_library(CCASimulator
            ${SourceDirectory}/Cell.cpp
            ${SourceDirectory}/CCASimulator.cpp
            ${SourceDirectory}/CCAFunctionEvents.cpp
            ${SourceDirectory}/ComputeCell.cpp
            ${SourceDirectory}/CyclicMemoryAllocator.cpp
            ${SourceDirectory}/VicinityMemoryAllocator.cpp
            ${SourceDirectory}/SinkCell.cpp
            ${SourceDirectory}/Terminator.cpp
            )

target_include_directories(CCASimulator PUBLIC 
                           ${IncludeDirectory}
                           ${IncludeDirectory}/Datastructure
                           )
                           
target_compile_options(CCASimulator PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter)

target_link_libraries(CCASimulator PUBLIC OpenMP::OpenMP_CXX)

# Add the executable for the cca_sssp.cpp source file
add_executable(${executable_name} cca_page_rank_nested_fixed_iterations.cpp)

target_include_directories(${executable_name} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${UtilitiesDirectory}
                           ${IncludeDirectory}
                           ${IncludeDirectory}/Datastructure
                           )

# Enable debugging
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

set_source_files_properties(cca_page_rank_nested_fixed_iterations.cpp PROPERTIES LANGUAGE CXX)
target_link_libraries(${executable_name} PRIVATE CCASimulator CCACommons HtreeNework)
