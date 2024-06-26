/*
BSD 3-Clause License

Copyright (c) 2023-2024, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <stdint.h>

// For older gcc compiler or something.
#ifndef u_int32_t
#define u_int32_t uint32_t
#endif

// ANSI escape codes for color to be used in std::cout statements, especially verification.
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_RESET "\x1b[0m"

// Set it to true/false
inline constexpr bool debug_code = false;

// Used for throttling.
// Compile with: -DTHROTTLE=true/false
inline constexpr bool throttling_switch = THROTTLE;

// Dump active status per cycle for each compute cells.
// Compile with: -DANIMATION=true/false
inline constexpr bool animation_switch = ANIMATION;

// Dump active status per cycle for all compute cells as a percentage.
// Compile with: -DACTIVE_PERCENT=true/false
inline constexpr bool active_percent_switch = ACTIVE_PERCENT;

// Compile with: -DACTIONQUEUESIZE=<int value> (use 64 or more/less or whatever).
inline constexpr u_int32_t action_queue_size = ACTIONQUEUESIZE;

// Compile with: -DDIFFUSE_QUEUE_SIZE=<int value> (use 4096 or more/less or whatever).
// TODO: BUG MAY HAPPEN --> If small chip size and diffuse_queue_size small then compute cells might
// deadlock.
inline constexpr u_int32_t diffuse_queue_size = DIFFUSE_QUEUE_SIZE;

// Compile with: -DVICINITY=<int value>
inline constexpr u_int32_t vicinity_radius = VICINITY;

// Compile with: -DWEIGHT=<bool value>
// Edge with weight or no weight.
inline constexpr bool weighted_edge = WEIGHT;

// Compile with: -DMIN_EDGES_PER_VERTEX=<int value>
// Used for the parent RPVO and its immediate childs. This is done to mimic std::vector like dynamic
// size functionality until such `list` like feature is implemented in the runtime. TODO
inline constexpr u_int32_t edges_min = MIN_EDGES_PER_VERTEX;

// Compile with: -DMAXEDGESPERVERTEX=<int value>
inline constexpr u_int32_t edges_max = MAXEDGESPERVERTEX;

// Compile with: -DGHOST_CHILDREN=<int value>
inline constexpr u_int32_t ghost_children_max = GHOST_CHILDREN;

// Use termination detection or do it without by only peeking at the active status of the cell
// queues and network queues.
// Compile with: -DTERMINATION=true/false
inline constexpr bool termination_switch = TERMINATION;

// Used for throttling. TODO: Make this sophisticated so that it adapts at runtime.
inline constexpr u_int32_t curently_congested_threshold = THROTTLE_CONGESTION_THRESHOLD; // cycles

// Number of total rhizomes per vertex.
inline constexpr u_int32_t rhizome_size = RHIZOME_SIZE;

// How many inbound edges before it switches to a new rhizome?
inline constexpr u_int32_t rhizome_inbound_degree_cutoff = RHIZOME_INDEGREE_CUTOFF;

// Either to split the queue into two: action and diffuse queue
// Or keep it one queue called the action queue
inline constexpr bool split_queues = SPLIT_QUEUES;

// To print to the output every output_skip_cycles's cycle
inline constexpr u_int32_t output_skip_cycles = 1;

// CPI
inline constexpr u_int32_t ADD_CPI = 1;
inline constexpr u_int32_t SUBT_CPI = 1;
inline constexpr u_int32_t MUL_CPI = 5;
inline constexpr u_int32_t DIV_CPI = 30;
inline constexpr u_int32_t LOAD_STORE_CPI = 1;

#endif // CONSTANTS_HPP
