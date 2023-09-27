#!/bin/bash

export OMP_NUM_THREADS=32

# Define possible values for each variable
THROTTLE_VALUES=("true" "false")
RECVBUFFSIZE_VALUES=("1" "2" "4" "8" "16")
TERMINATION_VALUES=("true" "false")


# REPO_PATH="/Users/bchandio/Documents/work/PhD/git_repos/CCA-Simulator"
# DATASET_PATH="/Users/bchandio/Documents/work/PhD/git_repos/CCA-Simulator/Papers/IPDPS_2024/Datasets"

# Parse command-line arguments
while getopts "r:d:" opt; do
    case "$opt" in
        r) REPO_PATH="$OPTARG";;
        d) DATASET_PATH="$OPTARG";;
        \?) echo "Usage: $0 [-r REPO_PATH] [-d DATASET_PATH]" >&2; exit 1;;
    esac
done

if [ -z "$REPO_PATH" ] || [ -z "$DATASET_PATH" ]; then
    echo "Both REPO_PATH and DATASET_PATH are required options."
    exit 1
fi

AAP_PATH="$REPO_PATH/Applications/Breadth_First_Search"
# Create a function to run cmake with the given parameters
run_cmake() {
    CC=gcc-13 CXX=g++-13 cmake -S "$AAP_PATH" -B build  -D ANIMATION=false -D VICINITY=8 -D MAXEDGESPERVERTEX=27 -D THROTTLE="$1" -D RECVBUFFSIZE="$2" -D TERMINATION="$3"
    cmake --build build -j 6
}

# Nested loops to iterate through all combinations
for THROTTLE in "${THROTTLE_VALUES[@]}"; do
    for RECVBUFFSIZE in "${RECVBUFFSIZE_VALUES[@]}"; do
        for TERMINATION in "${TERMINATION_VALUES[@]}"; do
            # Run cmake with the current combination of values
            run_cmake "$THROTTLE" "$RECVBUFFSIZE" "$TERMINATION"
            $REPO_PATH/Papers/IPDPS_2024/Runs/BFS/run_bfs_erdos_small.zsh -dataset "$DATASET_PATH"
        done
    done
done
