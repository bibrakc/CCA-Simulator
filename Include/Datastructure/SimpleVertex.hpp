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

#ifndef SimpleVertex_HPP
#define SimpleVertex_HPP

#include "Address.hpp"
#include "Constants.hpp"

template<typename Address_T>
struct Edge
{
    Address_T edge;
    u_int32_t weight;
};

inline constexpr u_int32_t max_distance = 999999;
inline constexpr u_int32_t edges_max = 30;
template<typename Address_T>
struct SimpleVertex
{
    u_int32_t id{};
    Edge<Address_T> edges[edges_max]{};
    u_int32_t number_of_edges{};
    u_int32_t sssp_distance;
    SimpleVertex(u_int32_t id_in)
        : id(id_in)
        , number_of_edges(0)
        , sssp_distance(max_distance)
    {
        /* std::cout << "SimpleVertex Constructor\n"; */
    }
    SimpleVertex()
    { /*  std::cout << "SimpleVertex default Constructor\n"; */
    }
    ~SimpleVertex()
    { /* std::cout << "SimpleVertex Destructor\n"; */
    }
};

#endif // SimpleVertex_HPP
