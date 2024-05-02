#!/bin/bash

#SBATCH -A YourAccount
#SBATCH -J sssp-lj-cca
#SBATCH -p general
#SBATCH -o lj-sssp_%j.txt
#SBATCH -e lj-sssp_%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=YourEmail
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=128
#SBATCH --time=02:30:00
#SBATCH --mem=140G

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

#CHIP_DIM_VALUES=("32" "64" "128")
#CHIP_DIM_VALUES=("16")
CHIP_DIM_VALUES=("$1")

declare -A MEMORY_CC
declare -A VICINITY
declare -A MAX_EDGES

MEMORY_CC[16]=29217728
VICINITY[16]=4
MAX_EDGES[16]=300

MEMORY_CC[32]=7308864
VICINITY[32]=5
MAX_EDGES[32]=100

MEMORY_CC[64]=1888608
VICINITY[64]=6
MAX_EDGES[64]=70

MEMORY_CC[128]=664304
VICINITY[128]=7
#VICINITY[128]=128
MAX_EDGES[128]=85

MIN_EDGES=50

declare -A RHIZOME_CUTOFF

# Not needed here
RHIZOME_CUTOFF[1]=13900

RHIZOME_CUTOFF[2]=6950

RHIZOME_CUTOFF[4]=3475

RHIZOME_CUTOFF[8]=1738

RHIZOME_CUTOFF[16]=869

# RHIZOMES=("16" "1" "2" "4" "8")
RHIZOMES=("1")
ACTION_QUEUE_SIZE=4096
DIFFUSE_QUEUE_SIZE=16384

ALLOCATOR_CONFIG=("VICINITY") # "RANDOM")

for ALLOCATOR in "${ALLOCATOR_CONFIG[@]}"; do
  for CHIP_DIM in "${CHIP_DIM_VALUES[@]}"; do
    memory=${MEMORY_CC[$CHIP_DIM]}
    vicinity=${VICINITY[$CHIP_DIM]}
    max_edges=${MAX_EDGES[$CHIP_DIM]}
    max_edges=100

    if [ "$CHIP_DIM" -eq 16 ]; then
    ACTION_QUEUE_SIZE=32768
    DIFFUSE_QUEUE_SIZE=32768
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
                 -memory_cc ${memory} -rhizomes ${RHIZOME} -cutoff ${CUTOFF} \
                 -action_queue_size ${ACTION_QUEUE_SIZE} \
                 -diffuse_queue_size ${DIFFUSE_QUEUE_SIZE} \
                 -script run_sssp_lj.zsh
    done
  done
done

