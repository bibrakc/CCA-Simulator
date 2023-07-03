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

#include "ComputeCell.hpp"
#include "TerminatorAction.hpp"

#include <cassert>

auto
Terminator::is_active() -> bool
{
    return (this->deficit == 0) ? false : true;
}

// Recieved an action. Increament my deficit.
void
Terminator::signal(ComputeCell& cc, const Address origin_addr_in)
{
    // this->deficit++;
    if (this->deficit == 0) {
        assert(this->parent == std::nullopt);

        this->parent = origin_addr_in;
    } else {
        // Send acknowledgement back to where the action came from
        TerminatorAction acknowledgement_action(
            origin_addr_in, this->my_object, actionType::terminator_acknowledgement_action);

        // TODO: put this counter in its own and separate betweek ack and nornal action
        cc.statistics.actions_created++;

        cc.statistics.actions_acknowledgement_created++;
        // Create Operon and put it in the task queue
        Operon operon_to_send =
            cc.construct_operon(cc.id, origin_addr_in.cc_id, acknowledgement_action);
        cc.task_queue.push(cc.send_operon(operon_to_send));
    }
}

// Make the object (vertex) inactive
void
Terminator::unsignal(ComputeCell& cc)
{
    if ((this->deficit == 0) && (this->parent != std::nullopt)) {
        if (this->parent.value().cc_id == cc.host_id) {
            // Simple decreament the deficit at the host.
            Object* obj = static_cast<Object*>(cc.get_object(this->parent.value()));
            obj->terminator.host_acknowledgement();
            this->parent = std::nullopt;
            std::cout << "Host Terminator Acknowledgement Sent!\n";
        } else {

            // Create an special acknowledgement action towards the parent in the
            // Dijkstra–Scholten spanning tree.
            TerminatorAction acknowledgement_action(this->parent.value(),
                                                    this->my_object,
                                                    actionType::terminator_acknowledgement_action);

            // TODO: put this counter in its own and separate betweek ack and nornal action
            cc.statistics.actions_created++;

            cc.statistics.actions_acknowledgement_created++;
            // Create Operon and put it in the task queue
            Operon operon_to_send =
                cc.construct_operon(cc.id, this->parent.value().cc_id, acknowledgement_action);
            cc.task_queue.push(cc.send_operon(operon_to_send));

            // Unset the parent
            this->parent = std::nullopt;
        }
    }
}

// Only when the terminator is created at the host and is used as root terminator for an
// application.
void
Terminator::host_signal()
{
    this->deficit++;
}
void
Terminator::host_acknowledgement()
{
    assert(this->deficit != 0);
    this->deficit--;
}

// Recieved an acknowledgement message back. Decreament my deficit.
void
Terminator::acknowledgement(ComputeCell& cc)
{
    if ((this->deficit == 0) && (this->parent != std::nullopt)) {
        if (this->parent.value().cc_id == cc.host_id) {
            // Simple decreament the deficit at the host.
            Object* obj = static_cast<Object*>(cc.get_object(this->parent.value()));
            obj->terminator.host_acknowledgement();
            this->parent = std::nullopt;
        } else {

            // Create an special acknowledgement action towards the parent in the
            // Dijkstra–Scholten spanning tree.
            TerminatorAction acknowledgement_action(this->parent.value(),
                                                    this->my_object,
                                                    actionType::terminator_acknowledgement_action);

            // TODO: put this counter in its own and separate betweek ack and nornal action
            cc.statistics.actions_created++;

            cc.statistics.actions_acknowledgement_created++;
            // Create Operon and put it in the task queue
            Operon operon_to_send =
                cc.construct_operon(cc.id, this->parent.value().cc_id, acknowledgement_action);
            cc.task_queue.push(cc.send_operon(operon_to_send));

            // Unset the parent
            this->parent = std::nullopt;
        }
        return;
    }

    this->deficit--;
    if (this->deficit == 0) {
        // Unset the parent and send an acknowledgement back to the parent.
        if (this->parent.value().cc_id == cc.host_id) {
            // Simple decreament the deficit at the host.
            Object* obj = static_cast<Object*>(cc.get_object(this->parent.value()));
            obj->terminator.host_acknowledgement();
            this->parent = std::nullopt;
        } else {

            // Create an special acknowledgement action towards the parent in the
            // Dijkstra–Scholten spanning tree.
            TerminatorAction acknowledgement_action(this->parent.value(),
                                                    this->my_object,
                                                    actionType::terminator_acknowledgement_action);

            // TODO: put this counter in its own and separate between ack and nornal action
            cc.statistics.actions_created++;

            cc.statistics.actions_acknowledgement_created++;
            // Create Operon and put it in the task queue
            Operon operon_to_send =
                cc.construct_operon(cc.id, this->parent.value().cc_id, acknowledgement_action);
            cc.task_queue.push(cc.send_operon(operon_to_send));

            // Unset the parent
            this->parent = std::nullopt;
        }
    }
}
