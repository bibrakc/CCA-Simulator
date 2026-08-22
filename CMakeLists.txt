# BSD 3-Clause License
#
# Copyright (c) 2023-2024, Bibrak Qamar
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

cmake_minimum_required(VERSION 3.20)
project(CCA-Simulator LANGUAGES CXX)

# ─── C++20 Requirement ───────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Put all executables in build/ root for convenience
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# ─── Sanitizer Support ───────────────────────────────────────────────────────
# Usage: cmake -DSANITIZER=address|thread|undefined|memory
if(DEFINED SANITIZER)
    if(SANITIZER STREQUAL "address")
        set(SANITIZER_FLAGS "-fsanitize=address,undefined -fno-omit-frame-pointer")
    elseif(SANITIZER STREQUAL "thread")
        set(SANITIZER_FLAGS "-fsanitize=thread -fno-omit-frame-pointer")
    elseif(SANITIZER STREQUAL "undefined")
        set(SANITIZER_FLAGS "-fsanitize=undefined -fno-omit-frame-pointer")
    elseif(SANITIZER STREQUAL "memory")
        set(SANITIZER_FLAGS "-fsanitize=memory -fno-omit-frame-pointer")
    else()
        message(FATAL_ERROR "Unknown SANITIZER value: ${SANITIZER}. Use: address, thread, undefined, or memory")
    endif()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZER_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SANITIZER_FLAGS}")
    message(STATUS "Sanitizer enabled: ${SANITIZER} (${SANITIZER_FLAGS})")
endif()

# ─── Configuration Options with Defaults ─────────────────────────────────────
if(NOT DEFINED THROTTLE)
    set(THROTTLE "true")
endif()
if(NOT DEFINED RECVBUFFSIZE)
    set(RECVBUFFSIZE 4)
endif()
if(NOT DEFINED ANIMATION)
    set(ANIMATION "false")
endif()
if(NOT DEFINED ACTIVE_PERCENT)
    set(ACTIVE_PERCENT "true")
endif()
if(NOT DEFINED ACTIONQUEUESIZE)
    set(ACTIONQUEUESIZE 64)
endif()
if(NOT DEFINED DIFFUSE_QUEUE_SIZE)
    set(DIFFUSE_QUEUE_SIZE 4096)
endif()
if(NOT DEFINED VICINITY)
    set(VICINITY 3)
endif()
if(NOT DEFINED WEIGHT)
    set(WEIGHT "true")
endif()
if(NOT DEFINED MIN_EDGES_PER_VERTEX)
    set(MIN_EDGES_PER_VERTEX 10)
endif()
if(NOT DEFINED MAXEDGESPERVERTEX)
    set(MAXEDGESPERVERTEX 30)
endif()
if(NOT DEFINED GHOST_CHILDREN)
    set(GHOST_CHILDREN 2)
endif()
if(NOT DEFINED TERMINATION)
    set(TERMINATION "false")
endif()
if(NOT DEFINED THROTTLE_CONGESTION_THRESHOLD)
    set(THROTTLE_CONGESTION_THRESHOLD 45)
endif()
if(NOT DEFINED RHIZOME_SIZE)
    set(RHIZOME_SIZE 2)
endif()
if(NOT DEFINED RHIZOME_INDEGREE_CUTOFF)
    set(RHIZOME_INDEGREE_CUTOFF 5000)
endif()
if(NOT DEFINED SPLIT_QUEUES)
    set(SPLIT_QUEUES "true")
endif()

# Collect all compile definitions into a list for targets
set(CCA_COMPILE_DEFINITIONS
    THROTTLE=${THROTTLE}
    RECVBUFFSIZE=${RECVBUFFSIZE}
    ANIMATION=${ANIMATION}
    ACTIVE_PERCENT=${ACTIVE_PERCENT}
    ACTIONQUEUESIZE=${ACTIONQUEUESIZE}
    DIFFUSE_QUEUE_SIZE=${DIFFUSE_QUEUE_SIZE}
    VICINITY=${VICINITY}
    WEIGHT=${WEIGHT}
    MIN_EDGES_PER_VERTEX=${MIN_EDGES_PER_VERTEX}
    MAXEDGESPERVERTEX=${MAXEDGESPERVERTEX}
    GHOST_CHILDREN=${GHOST_CHILDREN}
    TERMINATION=${TERMINATION}
    THROTTLE_CONGESTION_THRESHOLD=${THROTTLE_CONGESTION_THRESHOLD}
    RHIZOME_SIZE=${RHIZOME_SIZE}
    RHIZOME_INDEGREE_CUTOFF=${RHIZOME_INDEGREE_CUTOFF}
    SPLIT_QUEUES=${SPLIT_QUEUES}
)

message(STATUS "CCA-Simulator Configuration:")
message(STATUS "  THROTTLE: ${THROTTLE}")
message(STATUS "  RECVBUFFSIZE: ${RECVBUFFSIZE}")
message(STATUS "  ANIMATION: ${ANIMATION}")
message(STATUS "  ACTIVE_PERCENT: ${ACTIVE_PERCENT}")
message(STATUS "  ACTIONQUEUESIZE: ${ACTIONQUEUESIZE}")
message(STATUS "  DIFFUSE_QUEUE_SIZE: ${DIFFUSE_QUEUE_SIZE}")
message(STATUS "  VICINITY: ${VICINITY}")
message(STATUS "  WEIGHT: ${WEIGHT}")
message(STATUS "  MIN_EDGES_PER_VERTEX: ${MIN_EDGES_PER_VERTEX}")
message(STATUS "  MAXEDGESPERVERTEX: ${MAXEDGESPERVERTEX}")
message(STATUS "  GHOST_CHILDREN: ${GHOST_CHILDREN}")
message(STATUS "  TERMINATION: ${TERMINATION}")
message(STATUS "  THROTTLE_CONGESTION_THRESHOLD: ${THROTTLE_CONGESTION_THRESHOLD}")
message(STATUS "  RHIZOME_SIZE: ${RHIZOME_SIZE}")
message(STATUS "  RHIZOME_INDEGREE_CUTOFF: ${RHIZOME_INDEGREE_CUTOFF}")
message(STATUS "  SPLIT_QUEUES: ${SPLIT_QUEUES}")

# ─── Dependencies ────────────────────────────────────────────────────────────
find_package(OpenMP REQUIRED)

# ─── Directory Paths ─────────────────────────────────────────────────────────
set(CCA_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Source)
set(CCA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Include)
set(CCA_UTILITIES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Utilities)

# ─── Library: CCACommons ─────────────────────────────────────────────────────
add_library(CCACommons STATIC
    ${CCA_SOURCE_DIR}/Operon.cpp
    ${CCA_SOURCE_DIR}/types.cpp
)
target_include_directories(CCACommons PUBLIC
    ${CCA_INCLUDE_DIR}
    ${CCA_INCLUDE_DIR}/Datastructure
)
target_compile_definitions(CCACommons PUBLIC ${CCA_COMPILE_DEFINITIONS})
target_compile_options(CCACommons PRIVATE -Wall -Wextra)

# ─── Library: HtreeNetwork ───────────────────────────────────────────────────
add_library(HtreeNetwork STATIC
    ${CCA_SOURCE_DIR}/HtreeNode.cpp
    ${CCA_SOURCE_DIR}/HtreeNetwork.cpp
)
target_include_directories(HtreeNetwork PUBLIC
    ${CCA_INCLUDE_DIR}
)
target_compile_options(HtreeNetwork PRIVATE -Wall -Wextra)
target_link_libraries(HtreeNetwork PUBLIC CCACommons)

# ─── Library: CCASimulator ───────────────────────────────────────────────────
add_library(CCASimulator STATIC
    ${CCA_SOURCE_DIR}/Cell.cpp
    ${CCA_SOURCE_DIR}/CCASimulator.cpp
    ${CCA_SOURCE_DIR}/CCAFunctionEvents.cpp
    ${CCA_SOURCE_DIR}/ComputeCell.cpp
    ${CCA_SOURCE_DIR}/CyclicMemoryAllocator.cpp
    ${CCA_SOURCE_DIR}/VicinityMemoryAllocator.cpp
    ${CCA_SOURCE_DIR}/SinkCell.cpp
    ${CCA_SOURCE_DIR}/Terminator.cpp
)
target_include_directories(CCASimulator PUBLIC
    ${CCA_INCLUDE_DIR}
    ${CCA_INCLUDE_DIR}/Datastructure
)
target_compile_options(CCASimulator PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(CCASimulator PUBLIC OpenMP::OpenMP_CXX CCACommons)

# ─── Helper function to add a CCA application ────────────────────────────────
function(cca_add_application)
    cmake_parse_arguments(APP "" "NAME;SOURCE" "" ${ARGN})

    add_executable(${APP_NAME} ${APP_SOURCE})
    target_include_directories(${APP_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CCA_UTILITIES_DIR}
        ${CCA_INCLUDE_DIR}
        ${CCA_INCLUDE_DIR}/Datastructure
    )
    target_link_libraries(${APP_NAME} PRIVATE CCASimulator CCACommons HtreeNetwork)
endfunction()

# ─── Applications ────────────────────────────────────────────────────────────
add_subdirectory(Applications/Breadth_First_Search)
add_subdirectory(Applications/Breadth_First_Search_Rhizome)
add_subdirectory(Applications/Single_Source_Shortest_Path)
add_subdirectory(Applications/Single_Source_Shortest_Path_Rhizome)
add_subdirectory(Applications/Page_Rank_Fixed_Iterations)
add_subdirectory(Applications/Page_Rank_Fixed_Iterations_Rhizome)
add_subdirectory(Applications/Dynamic_Breadth_First_Search)
add_subdirectory(Applications/Streaming_Dynamic_Breadth_First_Search)
