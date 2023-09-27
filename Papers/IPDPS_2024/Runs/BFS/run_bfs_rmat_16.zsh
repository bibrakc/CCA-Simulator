#!/bin/zsh

# Define the path to the executable
EXECUTABLE="../../../../Applications/Breadth_First_Search/build/BFS_CCASimulator"


# Define the input file path
INPUT_FILE="../../Datasets/Synthetic/RMAT_ef_8_v_16.edgelist"

# Define other command-line arguments
GRAPH_TYPE="Erdos"
OUTPUT_DIR="./"
SHAPE="square"
ROOT_NODE="65535"
MEMORY_CC="10240"
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
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# Run the command
$EXECUTABLE -f $INPUT_FILE -g $GRAPH_TYPE -od $OUTPUT_DIR -s $SHAPE -root $ROOT_NODE -m $MEMORY_CC -hx $HX -hy $HY -hdepth $HDEPTH -hb $HB -route $ROUTE $VERIFY
