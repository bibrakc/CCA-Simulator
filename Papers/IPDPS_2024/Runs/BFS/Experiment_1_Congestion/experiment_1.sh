#!/bin/bash

# Define possible values for each variable
THROTTLE_VALUES=("true" "false")
RECVBUFFSIZE_VALUES=("1" "2" "4" "8" "16")
TERMINATION_VALUES=("true" "false")

# Create a function to run cmake with the given parameters
run_cmake() {
    AAP_PATH="../../../../../Applications/Breadth_First_Search"
    CC=gcc-13 CXX=g++-13 cmake -S "$AAP_PATH" -B build  -D ANIMATION=false -D VICINITY=8 -D MAXEDGESPERVERTEX=27 -D THROTTLE="$1" -D RECVBUFFSIZE="$2" -D TERMINATION="$3"
    cmake --build build -j 6
}

# Nested loops to iterate through all combinations
for THROTTLE in "${THROTTLE_VALUES[@]}"; do
    for RECVBUFFSIZE in "${RECVBUFFSIZE_VALUES[@]}"; do
        for TERMINATION in "${TERMINATION_VALUES[@]}"; do
            # Run cmake with the current combination of values
            run_cmake "$THROTTLE" "$RECVBUFFSIZE" "$TERMINATION"
            ../run_bfs_rmat_16.zsh
        done
    done
done
