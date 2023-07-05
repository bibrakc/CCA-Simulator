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

#ifndef TERMINATOR_HPP
#define TERMINATOR_HPP

//#include "TerminatorAction.hpp"

#include <optional>

// Forward declare.
class ComputeCell;

// Dijkstra–Scholten algorithm for termination detection
struct Terminator
{
    u_int32_t deficit;
    std::optional<Address> parent;

    // The address of the object of which this terminator is part of.
    Address my_object;

    auto is_active() -> bool;

    // Recieved an action. Increament my deficit.
    void signal(ComputeCell& cc, Address origin_addr_in);

    // Make the object (vertex) inactive
    void unsignal(ComputeCell& cc);

    // Only when the terminator is created at the host and is used as root terminator for an
    // application.
    void host_signal();
    void host_acknowledgement();

    // Recieved an acknowledgement message back. Decreament my deficit.
    void acknowledgement(ComputeCell& cc);

    // Reset the terminator to be used again. Useful in iterative algorithms like Page Rank.
    void reset();

    Terminator()
    {
        /* std::cout << "Terminator Constructor\n"; */
        this->deficit = 0;
        this->parent = std::nullopt;

        // this->my_object = std::nullopt;
    }
};

#endif // TERMINATOR_HPP
