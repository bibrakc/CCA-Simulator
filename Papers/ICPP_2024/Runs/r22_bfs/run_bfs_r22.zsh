#!/bin/zsh


# Define other command-line arguments
GRAPH_TYPE="RMAT"
OUTPUT_DIR="."
SHAPE="square"
ROOT_NODE="0" #"1345716"
MEMORY_CC="8388608" # This is just here not used as it will be initialized below form cmd
HX="128"
HY="128"
HDEPTH="0"
HB="0"
ROUTE="0"
NETWORK="1" # 0: Mesh, 1: Torus
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
    -dataset)
        DATASET_PATH="$2"
        shift 2
        ;;
    -network)
        NETWORK="$2"
        shift 2
        ;;
    -trail)
        TRAIL="$2"
        shift 2
        ;;
    *)
        echo "Unknown argument: $1"
        exit 1
        ;;
    esac
done

INPUT_FILE="$DATASET_PATH/muchiSim/kron22.edgelist"
# Define the path to the executable
EXECUTABLE="./build_${HX}_${TRAIL}/BFS_Rhizome_CCASimulator"

# Run the command
$EXECUTABLE -f $INPUT_FILE \
	    -g $GRAPH_TYPE \
	    -od $OUTPUT_DIR \
            -s $SHAPE -root $ROOT_NODE -m $MEMORY_CC \
            -hx $HX -hy $HY -hdepth $HDEPTH -hb $HB \
            -route $ROUTE -mesh $NETWORK \
            -shuffle $VERIFY -trail $TRAIL