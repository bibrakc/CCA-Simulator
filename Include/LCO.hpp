/*
BSD 3-Clause License

Copyright (c) 2024, Bibrak Qamar

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

#ifndef LCO_HPP
#define LCO_HPP

// #include "Action.hpp"
// #include "Function.hpp"

template<typename T, template<typename> class lcoType>
class LCO
{
  public:
    // Value.
    T val{};

    // Type of the LCO: AND type, future type, or some other.
    lcoType<T> lco;

    // The action to trigger when the LCO is ready.
    // TODO: Will implement this purist version where there is action and it is diffused in the
    // current CC and then triggered by the system
    // Action action;

    auto get_val() -> T { return val; }
    void set_val(T val_in) { this->val = val_in; }

    /*     LCO()
            : action(Address(0, 0, adressType::invalid_address),
                     Address(0, 0, adressType::invalid_address),
                     actionType::invalid_action,
                     false,
                     nullptr,
                     0,
                     0,
                     0,
                     0)
        {
        }
        LCO(T val_in)
            : val(val_in)
            , action(Address(0, 0, adressType::invalid_address),
                     Address(0, 0, adressType::invalid_address),
                     actionType::invalid_action,
                     false,
                     nullptr,
                     0,
                     0,
                     0,
                     0)
        {
        } */

    LCO() = default;

    LCO(T val_in)
        : val(val_in)
    {
    }
};

#endif // LCO_HPP
