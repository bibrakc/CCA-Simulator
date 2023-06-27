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

set(executable_name SSSP_CCASimulator)
project(${executable_name} LANGUAGES CXX)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
add_definitions(-DDEBUG_CODE=${DEBUG_CODE})
add_definitions(-DTHROTTLE=${THROTTLE})

find_package(OpenMP REQUIRED)

set(SourceDirectory ${CMAKE_CURRENT_SOURCE_DIR}/../../Source)

set(IncludeDirectory ${CMAKE_CURRENT_SOURCE_DIR}/../../Include)
set(UtilitiesDirectory ${CMAKE_CURRENT_SOURCE_DIR}/../../Utilities)

add_library(CCACommons
            ${SourceDirectory}/Operon.cpp
            ${SourceDirectory}/operators.cpp
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
            ${SourceDirectory}/SinkCell.cpp
            ${SourceDirectory}/Terminator.cpp
            )


target_include_directories(CCASimulator PUBLIC 
                           ${IncludeDirectory}
                           ${IncludeDirectory}/Datastructure
                           )
                           
target_compile_options(CCASimulator PRIVATE -Werror -Wall -Wextra -Wno-unused-parameter)

target_link_libraries(CCASimulator PUBLIC OpenMP::OpenMP_CXX)# CCACommons)

# Add the executable for the cca_sssp.cpp source file
add_executable(${executable_name} cca_sssp.cpp)

target_include_directories(${executable_name} PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}
                           ${UtilitiesDirectory}
                           ${IncludeDirectory}
                           ${IncludeDirectory}/Datastructure
                           )

set_source_files_properties(cca_sssp.cpp PROPERTIES LANGUAGE CXX)
target_link_libraries(${executable_name} PRIVATE CCASimulator CCACommons HtreeNework)
