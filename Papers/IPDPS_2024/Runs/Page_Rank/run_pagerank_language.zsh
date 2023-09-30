#!/bin/zsh

# Define the path to the executable
EXECUTABLE="./build/PageRank_Nested_Fixed_Iterations_CCASimulator"


# Define the input file path
# DATASET_PATH="/Users/bchandio/Documents/work/PhD/git_repos/CCA-Simulator/Papers/IPDPS_2024/Datasets"
# INPUT_FILE="$DATASET_PATH/Synthetic/tmp/Erdos-Renyi_directed_ef_8_v_9.edgelist"

# Define other command-line arguments
GRAPH_TYPE="Language"
OUTPUT_DIR="."
SHAPE="square"
ROOT_NODE="3"
MEMORY_CC="102400"
HX="128"
HY="128"
HDEPTH="0"
HB="0"
ROUTE="0"
VERIFY="-verify"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -hx)
            HX="$2"
            shift 2
            ;;
        -hy)
            HY="$2"
            shift 2
            ;;
        -m)
            MEMORY_CC="$2"
            shift 2
            ;;
        -iter) 
            ITER="$2"
            shift 2
            ;;    
        -dataset)
            DATASET_PATH="$2"
            shift 2
            ;;    
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

INPUT_FILE="$DATASET_PATH/Real/language.edgelist"
# Run the command
$EXECUTABLE -f $INPUT_FILE -g $GRAPH_TYPE -od $OUTPUT_DIR -s $SHAPE -root $ROOT_NODE -m $MEMORY_CC -hx $HX -hy $HY -hdepth $HDEPTH -hb $HB -route $ROUTE -iter $ITER
