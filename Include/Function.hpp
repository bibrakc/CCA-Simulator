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

#ifndef FUNCTION_HPP
#define FUNCTION_HPP

#include "Address.hpp"
#include "Enums.hpp"

#include <memory>

// Event Id of the function that is registered with the CCA: predicate, work, diffuse, and other
// runtime events like termination detection.
using CCAFunctionEvent = u_int32_t;

using ActionArgumentType = std::shared_ptr<char[]>;

// Returned by action functions (especially the work function).
// The second element ActionArgumentType are arguments to the diffuse function.
using Closure = std::pair<CCAFunctionEvent, ActionArgumentType>;

// Forward declare.
class ComputeCell;

// TODO: Maybe later convert these to `std::function`
using handler_func = Closure (*)(ComputeCell& cc,
                                 const Address addr,
                                 actionType action_type,
                                 const ActionArgumentType args);

// Recieved an acknowledgement message back. Decreament my deficit.
auto
terminator_acknowledgement_func(ComputeCell& cc,
                                const Address addr,
                                actionType action_type,
                                const ActionArgumentType args) -> Closure;

// null event.
auto
null_func(ComputeCell& /* cc */,
          const Address /* addr */,
          actionType /* action_type_in */,
          const ActionArgumentType /*args*/) -> Closure;

auto
null_true_func(ComputeCell& /* cc */,
               const Address /* addr */,
               actionType /* action_type_in */,
               const ActionArgumentType /*args*/) -> Closure;

auto
error_func(ComputeCell& /* cc */,
           const Address /* addr */,
           actionType /* action_type_in */,
           const ActionArgumentType /*args*/) -> Closure;

// This is what the action of `allocate_func` carries as payload.
struct AllocateArguments
{
    // Address of the vertex that requested memory allocation.
    Address src_vertex_addrs;
    // The index of the Future LCO within the vertex object's ghost_vertices. It will be sent back.
    u_int8_t ghost_vertices_future_lco_index;

    // Size in bytes requested.
    u_int32_t size_in_bytes{};

    // The continuation to trigger at the src_vertex_addrs's cc
    CCAFunctionEvent continuation;

    // Id of the vertex and total number of vertices in the graph. These are not really needed but
    // we are setting them for any potential debugging needs.
    u_int32_t vertex_id;
    u_int32_t total_number_of_vertices;
};

// This is what the action of `allocate_func` send back as payload.
struct AllocateReturnArguments
{
    // Address of the newly allocated memory.
    Address new_memory_addrs;

    // The index of the Future LCO within the vertex object's ghost_vertices. It will be sent back.
    u_int8_t ghost_vertices_future_lco_index;

    // TODO: currently assuming that it doesn't send error reply such that it was not able to
    // allocate.
};

// Allocate memory and return the address to the calling continuation defined by CCAFunctionEvent
// passed in arguments.
auto
allocate_func(ComputeCell& cc,
              const Address addr,
              actionType /* action_type_in */,
              const ActionArgumentType args) -> Closure;

#endif // FUNCTION_HPP
