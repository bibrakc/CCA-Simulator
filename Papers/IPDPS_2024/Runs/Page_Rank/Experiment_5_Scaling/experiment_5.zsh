#!/bin/zsh

# export OMP_NUM_THREADS=32

# Define possible values for each variable
THROTTLE_VALUES=("true")
RECVBUFFSIZE_VALUES=("4")
TERMINATION_VALUES=("false")

# Declare an associative array to store chip sizes and their congestion threshold cool down period for throttle
declare -A CHIP_SIZE

# Populate the array with chip dim and congestion threshold value
CHIP_SIZE[64]=90
CHIP_SIZE[91]=128
CHIP_SIZE[128]=181
CHIP_SIZE[181]=255

# REPO_PATH="/Users/bchandio/Documents/work/PhD/git_repos/CCA-Simulator"
# DATASET_PATH="/Users/bchandio/Documents/work/PhD/git_repos/CCA-Simulator/Papers/IPDPS_2024/Datasets"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -r)
            REPO_PATH="$2"
            shift 2
            ;;
        -d)
            DATASET_PATH="$2"
            shift 2
            ;;
        -cc)
            CC_EXE="$2"
            shift 2
            ;;
        -cxx)
            CXX_EXE="$2"
            shift 2
            ;;
        -vicinity)
            VICINITY_VAL="$2"
            shift 2
            ;;
        -maxedge)
            MAX_EDGE="$2"
            shift 2
            ;;    
        -script)
            SCRIPT_TO_RUN="$2"
            shift 2
            ;;             
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done


AAP_PATH="$REPO_PATH/Applications/Page_Rank_Nested_Fixed_Iterations"
# Create a function to run cmake with the given parameters
run_cmake() {
    CC="$CC_EXE" CXX="$CXX_EXE" cmake -S "$AAP_PATH" -B build  -D ANIMATION=false -D VICINITY="$VICINITY_VAL" -D MAXEDGESPERVERTEX="$MAX_EDGE" -D THROTTLE="$1" -D RECVBUFFSIZE="$2" -D TERMINATION="$3" -D THROTTLE_CONGESTION_THRESHOLD="$4" -D NESTEDITERATIONS="$5"
    cmake --build build -j 6
}

# Nested loops to iterate through all combinations
for THROTTLE in "${THROTTLE_VALUES[@]}"; do
    for RECVBUFFSIZE in "${RECVBUFFSIZE_VALUES[@]}"; do
        for TERMINATION in "${TERMINATION_VALUES[@]}"; do
            for DIM THROTTLE_CONGESTION_THRESHOLD in ${(kv)CHIP_SIZE}; do
                # Run cmake with the current combination of values
                run_cmake "$THROTTLE" "$RECVBUFFSIZE" "$TERMINATION" "$THROTTLE_CONGESTION_THRESHOLD" 10
                # Setting memory -m to 200KB because the 64x64 chip won't have enough memory to store large graphs.
                "$REPO_PATH/Papers/IPDPS_2024/Runs/Page_Rank/$SCRIPT_TO_RUN" -dataset "$DATASET_PATH" -hx "$DIM" -hy "$DIM" -m 204800 -iter 1
            done
        done
    done
done
