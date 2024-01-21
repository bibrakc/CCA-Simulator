#!/bin/zsh

# export OMP_NUM_THREADS=32

# Define possible values for each variable
THROTTLE_VALUES=("true")
RECVBUFFSIZE_VALUES=("4")
# TERMINATION_VALUES=("true" "false")
TERMINATION_VALUES=("false")
NETWORK_VALUES=("1") # 0: Mesh, 1: Torus

CHIP_SIZE_X="128"
CHIP_SIZE_Y="128"

# Function to calculate adjusted throttle time
calculate_throttle_time() {
    local baseThrottleTime=$1
    local bufferSize=$2
    local throttleReductionFactor=$3

    local adjustedThrottleTime
    adjustedThrottleTime=$((baseThrottleTime / (bufferSize ** throttleReductionFactor)))

    # Convert to integer
    adjustedThrottleTime=${adjustedThrottleTime%.*}

    # Return the adjusted throttle time
    echo $adjustedThrottleTime
}

# Function to calculate Pythagorean theorem
pythagorean() {
    if [[ $# -ne 3 ]]; then
        echo "Usage: $0 <side1> <side2> <netowork_type>"
        exit 1
    fi

    local side1=$1
    local side2=$2
    local netowork_type=$3

    # Calculate the hypotenuse
    local hypotenuse=$(echo "sqrt($side1^2 + $side2^2)" | bc -l)

    # Check if divide_flag is 1, then divide the hypotenuse by 2
    if [[ $netowork_type -eq 1 ]]; then
        hypotenuse=$(echo "$hypotenuse / 2" | bc -l)
    fi

    # Round up to the nearest integer using ceiling
    local hypotenuse=$(echo "scale=0; $hypotenuse / 1" | bc -l)

    # Return the result
    echo "$hypotenuse"
}
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

AAP_PATH="$REPO_PATH/Applications/Breadth_First_Search"
# Create a function to run cmake with the given parameters
run_cmake() {
    CC="$CC_EXE" CXX="$CXX_EXE" cmake -S "$AAP_PATH" -B build -D ACTIVE_PERCENT=false -D ANIMATION=false -D VICINITY="$VICINITY_VAL" -D MAXEDGESPERVERTEX="$MAX_EDGE" -D THROTTLE="$1" -D RECVBUFFSIZE="$2" -D TERMINATION="$3" -D THROTTLE_CONGESTION_THRESHOLD="$4"
    cmake --build build -j 6
}

# Nested loops to iterate through all combinations
for THROTTLE in "${THROTTLE_VALUES[@]}"; do
    for RECVBUFFSIZE in "${RECVBUFFSIZE_VALUES[@]}"; do
        for TERMINATION in "${TERMINATION_VALUES[@]}"; do
            for NETWORK in "${NETWORK_VALUES[@]}"; do

                ThrottleTime=$(pythagorean $CHIP_SIZE_X $CHIP_SIZE_X $NETWORK)

                # Calculate the adjusted throttle time based on the buffer size.
                # baseThrottleTime=181     # Throttle time when buffer size is 1
                # throttleReductionFactor=0.2  # Adjust this factor as needed
                # adjustedThrottleTime=$(calculate_throttle_time $baseThrottleTime $RECVBUFFSIZE $throttleReductionFactor)
                #adjustedThrottleTime=181

                echo "ThrottleTime: $ThrottleTime"
                # Run cmake with the current combination of values
                run_cmake "$THROTTLE" "$RECVBUFFSIZE" "$TERMINATION" "$ThrottleTime"
                "$REPO_PATH/Papers/ICS_2024/Runs/BFS/$SCRIPT_TO_RUN" -hx "$CHIP_SIZE_X" -hy "$CHIP_SIZE_Y" -dataset "$DATASET_PATH" -network "$NETWORK"
            done
        done
    done
done
