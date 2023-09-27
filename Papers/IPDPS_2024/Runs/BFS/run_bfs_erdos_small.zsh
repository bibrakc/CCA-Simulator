#!/bin/zsh

# Define the path to the executable
EXECUTABLE="./build/BFS_CCASimulator"


# Define the input file path
REPO_PATH="/Users/bchandio/Documents/work/PhD/git_repos/CCA-Simulator"
INPUT_FILE="$REPO_PATH/Papers/IPDPS_2024/Datasets/Synthetic/tmp/Erdos-Renyi_directed_ef_8_v_9.edgelist"

# Define other command-line arguments
GRAPH_TYPE="Erdos"
OUTPUT_DIR="."
SHAPE="square"
ROOT_NODE="193"
MEMORY_CC="10240"
HX="20"
HY="20"
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
        -repository)
            REPO_PATH="$2"
            shift 2
            ;;    
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Run the command
$EXECUTABLE -f $INPUT_FILE -g $GRAPH_TYPE -od $OUTPUT_DIR -s $SHAPE -root $ROOT_NODE -m $MEMORY_CC -hx $HX -hy $HY -hdepth $HDEPTH -hb $HB -route $ROUTE $VERIFY
