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

#ifndef CCA_SSSP_HPP
#define CCA_SSSP_HPP

#include "cmdparser.hpp"

void
configure_parser(cli::Parser& parser)
{
    parser.set_required<std::string>("f", "graphfile", "Path to the input data graph file");
    parser.set_required<std::string>("g",
                                     "graphname",
                                     "Name of the input graph used to set the name of the output "
                                     "file. Example: Erdos or anything");
    parser.set_required<std::string>("s", "shape", "Shape of the compute cell");
    parser.set_required<u_int32_t>("tv", "testvertex", "test vertex to print its sssp distance");
    parser.set_required<u_int32_t>(
        "root", "sssproot", "Root vertex for Single Source Shortest Path (SSSP)");
    parser.set_optional<u_int32_t>("m",
                                   "memory_per_cc",
                                   1 * 512 * 1024,
                                   "Memory per compute cell in bytes. Default is 0.5 MB");
    parser.set_optional<std::string>(
        "od", "outputdirectory", "./", "Path to the output file directory. Default: ./");

    parser.set_optional<u_int32_t>(
        "hx",
        "htree_x",
        3,
        "Rows of Cells that are served by a single end Htree node. hx must be an odd value");
    parser.set_optional<u_int32_t>(
        "hy",
        "htree_y",
        5,
        "Columns of Cells that are served by a single end Htree node. hy must be an odd value");

    parser.set_optional<u_int32_t>("hdepth",
                                   "htree_depth",
                                   0,
                                   "Depth of the Htree. This is the size of the Htree. \n\t0: No "
                                   "Htree\n\t1: 1 Htree\n\t2: 5 Htrees "
                                   "as it recurssively constructs more...");

    parser.set_optional<u_int32_t>(
        "hb",
        "hbandwidth_max",
        64,
        "Max possible lanes in the htree joints. The htree is recursively constructred with end "
        "nodes having 4 lanes (for sqaure cells) to the joint. Then in the next joint there are 8, "
        "then 16 and so on. There needs to be a max value to avoid exponential growth.");

    parser.set_optional<u_int32_t>("route", "routing_policy", 0, "Routing algorithm to use.");
}

#endif // CCA_SSSP_HPP
