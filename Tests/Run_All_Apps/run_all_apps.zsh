#!/bin/zsh

: <<'COMMENT_BLOCK'
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
COMMENT_BLOCK

CCA_SIMULATOR=../..
BUILD_DIR=build

CMAKE_INPUT_OPTIONS=(
   -B "${BUILD_DIR}"
   -D THROTTLE=true
   -D ANIMATION=false
   -D VICINITY=2
   -D TERMINATION=false
   -D MIN_EDGES_PER_VERTEX=5
   -D MAXEDGESPERVERTEX=10
   -D GHOST_CHILDREN=3
   -D THROTTLE_CONGESTION_THRESHOLD=22
   -D RECVBUFFSIZE=4
   -D ACTIONQUEUESIZE=2048
   -D RHIZOME_INDEGREE_CUTOFF=10
   -D SPLIT_QUEUES=true
)

HX=16
HY=16
#NETWORK=0 # Simple Mesh
NETWORK=1 # Torus-Mesh

echo "Compiling and running all applications to check for any compilation error or bugs introduced during development. This is a very basic test with a simple small graph."
rm -rf Output
mkdir Output

echo "Compiling Breadth_First_Search"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Breadth_First_Search "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Breadth_First_Search"
./build/BFS_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -hdepth 0 -hb 0 -route 0 -mesh ${NETWORK} -shuffle -verify

echo "Done!"

echo "Compiling Breadth_First_Search_Rhizome"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Breadth_First_Search_Rhizome "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Breadth_First_Search_Rhizome"
./build/BFS_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -hdepth 0 -hb 0 -route 0 -mesh ${NETWORK} -shuffle -verify

echo "Done!"

echo "Compiling Single_Source_Shortest_Path"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Single_Source_Shortest_Path "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Single_Source_Shortest_Path"
./build/SSSP_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} -shuffle -verify

echo "Done!"

echo "Compiling Single_Source_Shortest_Path_Rhizome"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Single_Source_Shortest_Path_Rhizome "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Single_Source_Shortest_Path_Rhizome"
./build/SSSP_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} -shuffle -verify

echo "Done!"

echo "Compiling Page_Rank_Fixed_Iterations"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Page_Rank_Fixed_Iterations "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Page_Rank_Fixed_Iterations"
./build/PageRank_Fixed_Iterations_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} -iter 5 -shuffle -verify

echo "Done!"

echo "Compiling Page_Rank_Fixed_Iterations_Rhizome"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Page_Rank_Fixed_Iterations_Rhizome "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Page_Rank_Fixed_Iterations_Rhizome"
./build/PageRank_Fixed_Iterations_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} -iter 9 -verify

echo "Done!"

echo "Compiling Dynamic_Breadth_First_Search"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Dynamic_Breadth_First_Search "${CMAKE_INPUT_OPTIONS[@]}"

cmake --build build -j 6

echo "Running Dynamic_Breadth_First_Search"
./build/Dynamic_BFS_CCASimulator -f ../../Input_Graphs/Dynamic/1K/streamingEdge_lowOverlap_lowBlockSizeVar_1000_nodes -g DG -od ./Output -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} -increments 10 -shuffle -verify

echo "Done!"
