#!/bin/bash

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

CCA_SIMULATOR=/N/u/bchandio/BigRed200/Research/git_repos/CCA-Simulator
DATASETS=/N/u/bchandio/BigRed200/Research/git_repos/Datasets/Dynamic/Edge_Sampling/500K
BUILD_DIR=build

export OMP_NUM_THREADS=128

CMAKE_INPUT_OPTIONS=(
   -B "${BUILD_DIR}"
   -D THROTTLE=true
   -D ANIMATION=false
   -D VICINITY=2
   -D TERMINATION=false
   -D MIN_EDGES_PER_VERTEX=25
   -D MAXEDGESPERVERTEX=25
   -D GHOST_CHILDREN=3
   -D RECVBUFFSIZE=4
   -D ACTIONQUEUESIZE=512
   -D DIFFUSE_QUEUE_SIZE=1024
   -D RHIZOME_INDEGREE_CUTOFF=10
   -D SPLIT_QUEUES=true
)

# Function to calculate Pythagorean theorem
pythagorean() {
   if [[ $# -ne 3 ]]; then
      echo "Usage: $0 <side1> <side2> <network_type>"
      exit 1
   fi

   local side1=$1
   local side2=$2
   local network_type=$3

   # Calculate the hypotenuse
   local hypotenuse=$(echo "sqrt($side1^2 + $side2^2)" | bc -l)

   # Check if network_type is 1, then divide the hypotenuse by 2
   if [[ $network_type -eq 1 ]]; then
      hypotenuse=$(echo "$hypotenuse / 2" | bc -l)
   fi

   # Round up to the nearest integer using ceiling
   local hypotenuse=$(echo "scale=0; $hypotenuse / 1" | bc -l)

   # Return the result
   echo "$hypotenuse"
}

#CHIP_DIM_VALUES=("16" "32" "64" "128")
CHIP_DIM_VALUES=("32")
#NETWORK_VALUES=("0" "1")
NETWORK_VALUES=("0")

declare -A MEMORY_CC
MEMORY_CC[16]=4194304
MEMORY_CC[32]=2097152 #1048576
MEMORY_CC[64]=262144
MEMORY_CC[128]=65536

echo "Compiling and running."
rm -rf Output
mkdir Output

for CHIP_DIM in "${CHIP_DIM_VALUES[@]}"; do
   memory=${MEMORY_CC[$CHIP_DIM]}
   for NETWORK in "${NETWORK_VALUES[@]}"; do
      ThrottleTime=$(pythagorean $CHIP_DIM $CHIP_DIM $NETWORK)

      echo "Compiling Streaming_Dynamic_Breadth_First_Search"
      rm -rf build

      CC=cc-13 CXX=c++ cmake -S ${CCA_SIMULATOR}/Applications/Streaming_Dynamic_Breadth_First_Search "${CMAKE_INPUT_OPTIONS[@]}" -D THROTTLE_CONGESTION_THRESHOLD=${ThrottleTime}

      cmake --build build -j 6

      echo "Running Streaming_Dynamic_Breadth_First_Search"
      ./build/Streaming_Dynamic_BFS_CCASimulator -f ${DATASETS}/simulated_blockmodel_graph_500000_nodes_edgeSample -g DG_edgeSample -od ./Output -s square -root 0 -m ${memory} -hx ${CHIP_DIM} -hy ${CHIP_DIM} -hdepth 0 -hb 0 -route 0 -mesh ${NETWORK} -increments 10 -shuffle
   done
done
echo "Done!"
