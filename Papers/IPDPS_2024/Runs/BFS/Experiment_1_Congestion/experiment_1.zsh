#!/bin/zsh

# export OMP_NUM_THREADS=32

# Define possible values for each variable
THROTTLE_VALUES=("true" "false")
RECVBUFFSIZE_VALUES=("1" "2" "4" "8" "16")
TERMINATION_VALUES=("true" "false")


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
    CC="$CC_EXE" CXX="$CXX_EXE" cmake -S "$AAP_PATH" -B build  -D ANIMATION=false -D VICINITY="$VICINITY_VAL" -D MAXEDGESPERVERTEX="$MAX_EDGE" -D THROTTLE="$1" -D RECVBUFFSIZE="$2" -D TERMINATION="$3" -D THROTTLE_CONGESTION_THRESHOLD="$4"
    cmake --build build -j 6
}

# Nested loops to iterate through all combinations
for THROTTLE in "${THROTTLE_VALUES[@]}"; do
    for RECVBUFFSIZE in "${RECVBUFFSIZE_VALUES[@]}"; do
        for TERMINATION in "${TERMINATION_VALUES[@]}"; do

            # Calculate the adjusted throttle time based on the buffer size.
            baseThrottleTime=181     # Throttle time when buffer size is 1
            throttleReductionFactor=0.2  # Adjust this factor as needed
            adjustedThrottleTime=$(calculate_throttle_time $baseThrottleTime $RECVBUFFSIZE $throttleReductionFactor)

            # Run cmake with the current combination of values
            run_cmake "$THROTTLE" "$RECVBUFFSIZE" "$TERMINATION" "$adjustedThrottleTime"
            "$REPO_PATH/Papers/IPDPS_2024/Runs/BFS/$SCRIPT_TO_RUN" -dataset "$DATASET_PATH"
        done
    done
done
