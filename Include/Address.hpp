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

// TODO: Try and see if `#pragma once` can be used here
#ifndef ADDRESS_HPP
#define ADDRESS_HPP

#include <iostream>

enum class adressType : u_int8_t
{
    host_address = 0,
    cca_address,
    invalid_address
};

struct Address
{
  public:
    // CCA address or Host address? Default is CCA address.
    adressType type;

    // Global ID of the compute cell where the address resides
    u_int32_t cc_id;
    // The offset to the memory of the compute cell
    u_int32_t addr;

    // Is true when this address is not pointing to any valid object
    // TODO: later can be used for garbage collection
    // bool is_valid;

    Address() = default;

    Address(const u_int32_t id, const u_int32_t address_in)
        : type(adressType::cca_address)
        , cc_id(id)
        , addr(address_in)
    {
    }

    Address(const u_int32_t id, const u_int32_t address_in, const adressType address_type_in)
        : type(address_type_in)
        , cc_id(id)
        , addr(address_in)
    {
    }

    friend auto operator<<(std::ostream& os, const Address& ad) -> std::ostream&
    {
        os << "(" << ad.cc_id << ", " << ad.addr << ")";
        return os;
    }

    friend auto operator==(const Address& lhs, const Address& rhs) -> bool
    {
        return (lhs.type == rhs.type && lhs.cc_id == rhs.cc_id && lhs.addr == rhs.addr);
    }
};

#endif // ADDRESS_HPP
