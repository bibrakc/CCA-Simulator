#!/bin/zsh

CCA_SIMULATOR=../..

echo "Compiling and running all applications to check for any compilation error or bugs introduced during development. This is a very basic test with a simple small graph."
rm -rf Output
mkdir Output

echo "Compiling Breadth_First_Search"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Breadth_First_Search -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Breadth_First_Search"
./build/BFS_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -shuffle -verify

echo "Done!"

echo "Compiling Breadth_First_Search_Rhizome"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Breadth_First_Search_Rhizome -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Breadth_First_Search_Rhizome"
./build/BFS_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -shuffle -verify

echo "Done!"


echo "Compiling Single_Source_Shortest_Path"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Single_Source_Shortest_Path -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Single_Source_Shortest_Path"
./build/SSSP_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -shuffle -verify

echo "Done!"

echo "Compiling Single_Source_Shortest_Path_Rhizome"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Single_Source_Shortest_Path_Rhizome -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Single_Source_Shortest_Path_Rhizome"
./build/SSSP_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -shuffle -verify

echo "Done!"

echo "Compiling Page_Rank_Fixed_Iterations"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Page_Rank_Fixed_Iterations -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Page_Rank_Fixed_Iterations"
./build/PageRank_Fixed_Iterations_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 0 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -iter 5 -shuffle -verify

echo "Done!"

echo "Compiling Page_Rank_Fixed_Iterations_Rhizome"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Page_Rank_Fixed_Iterations_Rhizome -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Page_Rank_Fixed_Iterations_Rhizome"
./build/PageRank_Fixed_Iterations_Rhizome_CCASimulator -f ../../Input_Graphs/Erdos-Renyi_directed_ef_16_v_11.edgelist -g Erdos -od ./Output -s square -root 3 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -iter 9 -verify

echo "Done!"

echo "Compiling Dynamic_Breadth_First_Search"
rm -rf build

CC=gcc-13 CXX=g++-13 cmake -S ${CCA_SIMULATOR}/Applications/Dynamic_Breadth_First_Search -B build -D THROTTLE=true -D ANIMATION=false -D VICINITY=2 -D TERMINATION=false -D MAXEDGESPERVERTEX=10 -D THROTTLE_CONGESTION_THRESHOLD=22 -D RECVBUFFSIZE=4 -D ACTIONQUEUESIZE=256 -D RHIZOME_INDEGREE_CUTOFF=10

cmake --build build -j 6

echo "Running Dynamic_Breadth_First_Search"
./build/Dynamic_BFS_CCASimulator -f ../../Input_Graphs/Dynamic/1K/streamingEdge_lowOverlap_lowBlockSizeVar_1000_nodes -g DG -od ./Output -s square -root 0 -m 90000 -hx 32 -hy 32 -hdepth 0 -hb 0 -route 0 -mesh 1 -increments 10 -shuffle -verify

echo "Done!"