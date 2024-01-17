/*
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

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

// For older gcc compiler or something.
#ifndef u_int32_t
#define u_int32_t uint32_t
#endif

// Compile with: -DVICINITY=<int value>
inline constexpr u_int32_t vicinity_radius = VICINITY;

// Compile with: -DMAXEDGESPERVERTEX=<int value>
inline constexpr u_int32_t edges_max = MAXEDGESPERVERTEX;

// Use termination detection or do it without by only peeking at the active status of the cell
// queues and network queues.
// Compile with: -DTERMINATION=true/false
inline constexpr bool termination_switch = TERMINATION;

// Used for throttling. TODO: Make this sophisticated so that it adapts at runtime.
constexpr u_int32_t curently_congested_threshold = THROTTLE_CONGESTION_THRESHOLD; // cycles

#endif // CONSTANTS_HPP
