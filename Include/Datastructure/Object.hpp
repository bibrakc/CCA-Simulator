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

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include "Action.hpp"
#include "Address.hpp"

// #include <cassert>

class TerminatorAction : public Action
{
  public:
    TerminatorAction(const Address obj_addr_in,
                     const Address origin_addr_in,
                     const actionType action_type_in)
    {
        // This is the address to which the action will be sent
        this->obj_addr = obj_addr_in;

        // This is the type of the action, most likely:
        // `actionType::terminator_acknowledgement_action`
        this->action_type = action_type_in;

        // Not really needed
        this->origin_addr = origin_addr_in;
    }

    ~TerminatorAction() override {}
};

// Dijkstra–Scholten algorithm for termination detection
struct Terminator
{
    u_int32_t deficit;
    std::optional<Address> parent;

    // The address of the object of which this terminator is part of.
    // TODO: This is not really needed. Check later.
    Address my_object;

    // Recieved an action. Increament my deficit.
    void signal(const Address origin_addr_in)
    {
        if (this->deficit == 0) {
            this->parent = origin_addr_in;
            this->deficit++;
        } else {
            // Send acknowledgement back to where the action came from
            TerminatorAction acknowledgement_action(
                origin_addr_in, this->my_object, actionType::terminator_acknowledgement_action);

            // Put a task in the task queue of the CC
            Actually send !
        }
    }

    // Recieved an acknowledgement message back. Decreament my deficit.
    void acknowledgement(ComputeCell& cc)
    {
        assert(this->deficit != 0);

        this->deficit--;
        if (this->deficit == 0) {
            // Unset the parent and send an acknowledgement back to the parent

            // Create an special acknowledgement action towards the parent in the Dijkstra–Scholten
            // spanning tree.
            TerminatorAction acknowledgement_action(this->parent.value(),
                                                    this->my_object,
                                                    actionType::terminator_acknowledgement_action);

            // Create Operon and put it in the task queue
            Operon operon_to_send =
                construct_operon(this->parent.value().cc_id, acknowledgement_action);
            cc.task_queue.push(send_operon(cc, operon_to_send));

            // Unset the parent
            this->parent = std::nullopt;
            Actually send !
        }
    }

    Terminator()
    {
        /* std::cout << "Terminator Constructor\n"; */
        this->deficit = 0;
        this->parent = std::nullopt;

        // this->my_object = std::nullopt;
    }
};

struct Object
{
    // Terminator
    Terminator terminator;

    // Garbage collection

    // Type tag?

    Object()
    { /* std::cout << "Object Constructor\n"; */
    }
};

#endif // OBJECT_HPP
