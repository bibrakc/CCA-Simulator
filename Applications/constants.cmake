# BSD 3-Clause License
# 
# Copyright (c) 2024, Bibrak Qamar
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


message(STATUS "Configuring constants as:")
if(NOT DEFINED THROTTLE)
    set(THROTTLE "true")
endif()
add_definitions(-DTHROTTLE=${THROTTLE})
message(STATUS "\tTHROTTLE: ${THROTTLE}")

if(NOT DEFINED RECVBUFFSIZE)
    set(RECVBUFFSIZE 4)
endif()
add_definitions(-DRECVBUFFSIZE=${RECVBUFFSIZE})
message(STATUS "\tRECVBUFFSIZE: ${RECVBUFFSIZE}")

if(NOT DEFINED ANIMATION)
    set(ANIMATION "false")
endif()
add_definitions(-DANIMATION=${ANIMATION})
message(STATUS "\tANIMATION: ${ANIMATION}")

if(NOT DEFINED ACTIVE_PERCENT)
    set(ACTIVE_PERCENT "true")
endif()
add_definitions(-DACTIVE_PERCENT=${ACTIVE_PERCENT})
message(STATUS "\tACTIVE_PERCENT: ${ACTIVE_PERCENT}")

if(NOT DEFINED ACTIONQUEUESIZE)
    set(ACTIONQUEUESIZE 64)
endif()
add_definitions(-DACTIONQUEUESIZE=${ACTIONQUEUESIZE})
message(STATUS "\tACTIONQUEUESIZE: ${ACTIONQUEUESIZE}")

if(NOT DEFINED VICINITY)
    set(VICINITY 3)
endif()
add_definitions(-DVICINITY=${VICINITY})
message(STATUS "\tVICINITY: ${VICINITY}")

if(NOT DEFINED MAXEDGESPERVERTEX)
    set(MAXEDGESPERVERTEX 30)
endif()
add_definitions(-DMAXEDGESPERVERTEX=${MAXEDGESPERVERTEX})
message(STATUS "\tMAXEDGESPERVERTEX: ${MAXEDGESPERVERTEX}")

if(NOT DEFINED TERMINATION)
    set(TERMINATION "false")
endif()
add_definitions(-DTERMINATION=${TERMINATION})
message(STATUS "\tTERMINATION: ${TERMINATION}")

if(NOT DEFINED THROTTLE_CONGESTION_THRESHOLD)
    set(THROTTLE_CONGESTION_THRESHOLD 45)
endif()
add_definitions(-DTHROTTLE_CONGESTION_THRESHOLD=${THROTTLE_CONGESTION_THRESHOLD})
message(STATUS "\tTHROTTLE_CONGESTION_THRESHOLD: ${THROTTLE_CONGESTION_THRESHOLD}")

# Rhizome related
if(NOT DEFINED RHIZOME_SIZE)
    set(RHIZOME_SIZE 2)
endif()
add_definitions(-DRHIZOME_SIZE=${RHIZOME_SIZE})
message(STATUS "\tRHIZOME_SIZE: ${RHIZOME_SIZE}")

if(NOT DEFINED RHIZOME_INDEGREE_CUTOFF)
    set(RHIZOME_INDEGREE_CUTOFF 5000)
endif()
add_definitions(-DRHIZOME_INDEGREE_CUTOFF=${RHIZOME_INDEGREE_CUTOFF})
message(STATUS "\tRHIZOME_INDEGREE_CUTOFF: ${RHIZOME_INDEGREE_CUTOFF}")

# Pruning and computation/communication overlap related
if(NOT DEFINED SPLIT_QUEUES)
    set(SPLIT_QUEUES "true")
endif()
add_definitions(-DSPLIT_QUEUES=${SPLIT_QUEUES})
message(STATUS "\tSPLIT_QUEUES: ${SPLIT_QUEUES}")