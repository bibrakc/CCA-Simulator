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

#ifndef OPERON_HPP
#define OPERON_HPP

#include "Action.hpp"
#include "ComputeCell.hpp"
#include "Object.hpp"
#include "Task.hpp"
#include "types.hpp"

#include <utility>

// cc_id, Action
typedef std::pair<u_int32_t, Action> Operon;

// For Htree routing. At the end node of Htree that connects to a sink cell take the Coordinates out
// and send the simple Operon to the CCA chip though the sink cell
typedef std::pair<Coordinates, Operon> CoordinatedOperon;

/// Send an Operon. Create a task that when invoked on a Compute Cell it simply puts the operon on
// the `staging_operon_from_logic`
inline Task
send_operon(ComputeCell& cc, Operon operon_in)
{

    // Increament the deficit for termination detection if actionType !=
    // terminator_acknowledgement_action
    actionType action_type = operon_in.second.action_type;
    if (action_type == actionType::application_action) {
        Address addr = operon_in.second.origin_addr;
        Object* obj = static_cast<Object*>(cc.get_object(addr));
        obj->terminator.deficit++;
    }

    return std::pair<taskType, Task_func>(
        taskType::send_operon_task_type, Task_func([&cc, operon_in]() {
            // Bug check
            if (cc.staging_operon_from_logic != std::nullopt) {
                std::cerr << "Bug! cc: " << cc.id
                          << " staging_operon_from_logic buffer is full! The program shouldn't "
                             "have come "
                             "to send_operon\n";
                exit(0);
            }

            // Debug prints
            if constexpr (debug_code) {
                std::cout << "Sending operon from cc: " << cc.id << " to cc: " << operon_in.first
                          << "\n";
            }

            // Actual work of sending
            cc.staging_operon_from_logic = operon_in;
        }));
}

// send_acknoledgement_operon

inline Operon
construct_operon(const u_int32_t cc_id, const Action& action)
{
    return std::pair<u_int32_t, Action>(cc_id, action);
}

#endif // OPERON_HPP
