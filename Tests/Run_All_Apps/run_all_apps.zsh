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
   -D MIN_EDGES_PER_VERTEX=10
   -D MAXEDGESPERVERTEX=15
   -D GHOST_CHILDREN=3
   -D THROTTLE_CONGESTION_THRESHOLD=22
   -D RECVBUFFSIZE=4
   -D ACTIONQUEUESIZE=2048
   -D RHIZOME_INDEGREE_CUTOFF=10
   -D SPLIT_QUEUES=true
)

HX=16
HY=16
NETWORK=1 # Torus-Mesh

FAILURES=0

echo "Compiling and running all applications to check for any compilation error or bugs introduced during development. This is a very basic test with a simple small graph."
START_TIME=$SECONDS
rm -rf ${BUILD_DIR}
mkdir -p Output

# Auto-detect GCC for OpenMP support (Apple Clang lacks OpenMP)
if command -v gcc-13 &>/dev/null; then
    export CC=gcc-13 CXX=g++-13
elif command -v gcc-15 &>/dev/null; then
    export CC=gcc-15 CXX=g++-15
elif command -v gcc-14 &>/dev/null; then
    export CC=gcc-14 CXX=g++-14
fi

echo "Building all applications with unified build system..."
cmake -S ${CCA_SIMULATOR} "${CMAKE_INPUT_OPTIONS[@]}"
if [ $? -ne 0 ]; then
    echo "ERROR: CMake configuration failed!"
    exit 1
fi

cmake --build ${BUILD_DIR} -j $(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
if [ $? -ne 0 ]; then
    echo "ERROR: Build failed!"
    exit 1
fi

echo "Build successful. Running tests..."
echo ""

# Helper function to run a test and check result
run_test() {
    local name=$1
    shift
    echo "Running ${name}..."
    local output
    output=$("$@" 2>&1)
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        echo "ERROR: ${name} exited with code ${exit_code}"
        FAILURES=$((FAILURES + 1))
        return 1
    fi
    if echo "$output" | grep -q "Verification Failed\|Verification Unsuccessful"; then
        echo "ERROR: ${name} verification FAILED!"
        echo "$output" | grep -A1 "Verification"
        FAILURES=$((FAILURES + 1))
        return 1
    fi
    echo "$output" | grep "Verification"
    echo "Done!"
    echo ""
}

run_test "Breadth_First_Search" \
    ./${BUILD_DIR}/BFS_CCASimulator \
    -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -hdepth 0 -hb 0 -route 0 -mesh ${NETWORK} \
    -shuffle -verify

run_test "Breadth_First_Search_Rhizome" \
    ./${BUILD_DIR}/BFS_Rhizome_CCASimulator \
    -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -hdepth 0 -hb 0 -route 0 -mesh ${NETWORK} \
    -shuffle -verify

run_test "Single_Source_Shortest_Path" \
    ./${BUILD_DIR}/SSSP_CCASimulator \
    -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} \
    -shuffle -verify

run_test "Single_Source_Shortest_Path_Rhizome" \
    ./${BUILD_DIR}/SSSP_Rhizome_CCASimulator \
    -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} \
    -shuffle -verify

run_test "Page_Rank_Fixed_Iterations" \
    ./${BUILD_DIR}/PageRank_Fixed_Iterations_CCASimulator \
    -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} \
    -iter 5 -shuffle -verify

run_test "Page_Rank_Fixed_Iterations_Rhizome" \
    ./${BUILD_DIR}/PageRank_Fixed_Iterations_Rhizome_CCASimulator \
    -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output \
    -s square -root 3 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} \
    -iter 9 -verify

run_test "Dynamic_Breadth_First_Search" \
    ./${BUILD_DIR}/Dynamic_BFS_CCASimulator \
    -f ../../Input_Graphs/Dynamic/1K/streamingEdge_lowOverlap_lowBlockSizeVar_1000_nodes -g DG -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK} \
    -increments 10 -shuffle -verify

NETWORK_STREAMING=0 # Streaming currently only supported for pure Mesh.

run_test "Streaming_Dynamic_Breadth_First_Search" \
    ./${BUILD_DIR}/Streaming_Dynamic_BFS_CCASimulator \
    -f ../../Input_Graphs/Dynamic/1K/streamingEdge_lowOverlap_lowBlockSizeVar_1000_nodes -g DG_Streaming -od ./Output \
    -s square -root 0 -m 90000 -hx ${HX} -hy ${HY} -route 0 -mesh ${NETWORK_STREAMING} \
    -increments 10 -shuffle -verify

echo ""
echo "=========================================="
ELAPSED=$((SECONDS - START_TIME))
TOTAL_TESTS=8
PASSED=$((TOTAL_TESTS - FAILURES))
if [ $FAILURES -eq 0 ]; then
    echo "ALL TESTS PASSED (${PASSED}/${TOTAL_TESTS}) in ${ELAPSED}s"
    exit 0
else
    echo "FAILURES: ${FAILURES}/${TOTAL_TESTS} test(s) failed! (${ELAPSED}s)"
    exit 1
fi
