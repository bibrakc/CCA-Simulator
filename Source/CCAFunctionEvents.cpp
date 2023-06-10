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

#include "CCAFunctionEvents.hpp"
#include "Address.hpp"
#include "Function.hpp"
#include "Object.hpp"

#include <cassert>
#include <vector>

handler_func
FunctionEventManager::get_acknowledgement_event_handler()
{
    return this->event_handlers[this->acknowledgement_event_id];
}

CCAFunctionEvent
FunctionEventManager::register_function_event(handler_func function_event_handler)
{
    assert(this->next_available_event_id == this->event_handlers.size());

    CCAFunctionEvent current_function_event_id = this->next_available_event_id;
    this->event_handlers.push_back(function_event_handler);
    this->next_available_event_id++;

    return current_function_event_id;
}

handler_func
FunctionEventManager::get_function_event_handler(CCAFunctionEvent function_event_in)
{
    assert(function_event_in < this->event_handlers.size());

    return this->event_handlers[function_event_in];
}

extern CCAFunctionEvent sssp_predicate;
extern CCAFunctionEvent sssp_work;
extern CCAFunctionEvent sssp_diffuse;

