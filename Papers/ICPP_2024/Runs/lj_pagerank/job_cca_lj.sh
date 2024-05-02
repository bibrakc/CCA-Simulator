#!/bin/bash

#SBATCH -A YourAccount
#SBATCH -J pr-lj-cca
#SBATCH -p general
#SBATCH -o lj-pr_%j.txt
#SBATCH -e lj-pr_%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=YourEmail
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=128
#SBATCH --time=01:25:00
#SBATCH --mem=100G

#Load any modules that your program needs
#module load modulename

repository=PATH-TO/CCA-Simulator
dataset=PATH-TO/Datasets

#change directory to:
cd ${repository}/Papers/ICPP_2024/Runs/

# Script to execute
exe=experiment_5.zsh

#Define the list of number of processes
threads=128
export OMP_NUM_THREADS=${threads}

# CHIP_DIM_VALUES=("32" "64" "128" "16")
#CHIP_DIM_VALUES=("16")
CHIP_DIM_VALUES=("$1")

declare -A MEMORY_CC
declare -A VICINITY
declare -A MAX_EDGES

MEMORY_CC[16]=22217728
VICINITY[16]=4
MAX_EDGES[16]=300

MEMORY_CC[32]=8108864
VICINITY[32]=5
MAX_EDGES[32]=300

MEMORY_CC[64]=2988608
VICINITY[64]=6
MAX_EDGES[64]=150

MEMORY_CC[128]=454304
VICINITY[128]=7
#VICINITY[128]=128
MAX_EDGES[128]=100

MIN_EDGES=60

declare -A RHIZOME_CUTOFF

# Not needed here
RHIZOME_CUTOFF[1]=13900

RHIZOME_CUTOFF[2]=6950

RHIZOME_CUTOFF[4]=3475

RHIZOME_CUTOFF[8]=1738

RHIZOME_CUTOFF[16]=869


#RHIZOMES=("16" "1" "4")
RHIZOMES=("16")
ACTION_QUEUE_SIZE=32768

ALLOCATOR_CONFIG=("VICINITY") # "RANDOM")

for ALLOCATOR in "${ALLOCATOR_CONFIG[@]}"; do
  for CHIP_DIM in "${CHIP_DIM_VALUES[@]}"; do
    memory=${MEMORY_CC[$CHIP_DIM]}
    vicinity=${VICINITY[$CHIP_DIM]}
    max_edges=${MAX_EDGES[$CHIP_DIM]}
    
    max_edges=100
    vicinity=2

    if [ "$CHIP_DIM" -eq 16 ]; then
    ACTION_QUEUE_SIZE=32768
    fi

    if [ "$ALLOCATOR" = "RANDOM" ]; then
        vicinity=${CHIP_DIM}
    fi

    for RHIZOME in "${RHIZOMES[@]}"; do
	CUTOFF=${RHIZOME_CUTOFF[$RHIZOME]}
            srun -N 1 -n 1 -c ${threads} ${exe} -r ${repository} -d ${dataset} \
                 -cc cc -cxx c++ -vicinity ${vicinity} \
                 -minedge ${MIN_EDGES} -maxedge ${max_edges} \
                 -chip_dim ${CHIP_DIM} \
                 -memory_cc ${memory} -rhizomes ${RHIZOME} -cutoff ${CUTOFF} -action_queue_size ${ACTION_QUEUE_SIZE} -script run_pagerank_lj.zsh
    done
  done
done

