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
#include "Enums.hpp"

enum class actionType : u_int32_t
{
    internal_action = 0,
    application_action,
    actionType_count
};

class Action
{
  public:
    // Type of the action: application type, internal runtime work action,
    // or any other
    actionType action_type;

    // Sets to `true` when all dependencies for this action are satisfied
    // and this action is ready to be executed
    // TODO: Think about how to use it in complex settings
    bool is_ready;

    // Number of arguments to the action function
    int nargs;

    // Payload that contains the data like the arguments to the action function
    std::shared_ptr<int[]> args;

    // Memory location of the object for which this action is destined
    // std::shared_ptr<Address> vertex_addr;
    Address vertex_addr;

    // Predicate
    eventId predicate;

    // Work function that does some computation and may change the state
    // of the object for which this action is destined
    eventId work;

    // Generate actions along the edges for the diffusion
    eventId diffuse;

    // We can't just delete the args here since we don't know
    ~Action()
    { /* std::cout << "In Action destructor" << std::endl; */
    }
};
#endif // ACTION_HPP