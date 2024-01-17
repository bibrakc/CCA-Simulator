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

#ifndef CCA_FUNCTION_EVENTS_HPP
#define CCA_FUNCTION_EVENTS_HPP

#include "Function.hpp"
#include <vector>

struct FunctionEventManager
{
    // Stores the functions that are called from within an action invokation. These may include
    // predicate, work, diffuse, and other runtime related work such as termination detection logic.
    std::vector<handler_func> event_handlers;

    // Each event gets its unique id that is used by the end user or runtime to create actions that
    // will call events.
    CCAFunctionEvent next_available_event_id{ 0 };

    // Used for special actions such as terminator that don't call predicate, work, and diffuse.
    CCAFunctionEvent null_event_id;

    // Used for special actions such as edge insertion with continuation for dynamic graphs that
    // don't need to have predicate,and diffuse.
    CCAFunctionEvent null_event_true_id;

    // Special system events:
    // Acknowledgement event id for termination detection.
    CCAFunctionEvent acknowledgement_event_id;

    handler_func get_acknowledgement_event_handler();

    auto register_function_event(handler_func function_event_handler) -> CCAFunctionEvent;

    handler_func get_function_event_handler(CCAFunctionEvent function_event_in);

    FunctionEventManager()
        : event_handlers()
        , null_event_id(register_function_event(null_func))
        , null_event_true_id(register_function_event(null_true_func))
        , acknowledgement_event_id(register_function_event(terminator_acknowledgement_func))
    {
    }
};

#endif // CCA_FUNCTION_EVENTS_HPP
