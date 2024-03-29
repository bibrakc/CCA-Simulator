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

#ifndef ACTION_HPP
#define ACTION_HPP

#include "Address.hpp"
#include "Function.hpp"

class Action
{
  public:
    // Type of the action: application type, internal runtime work action like terminator_action,
    // host type like germinate, or any other
    actionType action_type;

    // Sets to `true` when all dependencies for this action are satisfied
    // and this action is ready to be executed
    // TODO: Think about how to use it in complex settings
    bool is_ready;

    // Number of arguments to the action function
    // Not needed anymore.
    /* int nargs; */

    // Payload that contains the data like the arguments to the action function
    ActionArgumentType args;

    // Memory location of the object for which this action is destined
    Address obj_addr;

    // Memory location of the object from which this action originated. Used for termination
    // detection.
    Address origin_addr;

    // Predicate
    CCAFunctionEvent predicate;

    // Work function that does some computation and may change the state
    // of the object for which this action is destined
    CCAFunctionEvent work;

    // Generate actions along the edges for the diffusion
    CCAFunctionEvent diffuse_predicate;
    CCAFunctionEvent diffuse;

    Action(const Address destination_vertex_addr_in,
           const Address origin_vertex_addr_in,
           actionType type,
           const bool ready,
           const ActionArgumentType args_in,
           CCAFunctionEvent predicate_in,
           CCAFunctionEvent work_in,
           CCAFunctionEvent diffuse_predicate_in,
           CCAFunctionEvent diffuse_in)
    {
        this->obj_addr = destination_vertex_addr_in;
        this->origin_addr = origin_vertex_addr_in;

        this->action_type = type;
        this->is_ready = ready;

        this->args = args_in;

        this->predicate = predicate_in;
        this->work = work_in;
        this->diffuse_predicate = diffuse_predicate_in;
        this->diffuse = diffuse_in;
    }

    ~Action()
    { /* std::cout << "Action class destructor" << std::endl;  */
    }
};

#endif // ACTION_HPP
