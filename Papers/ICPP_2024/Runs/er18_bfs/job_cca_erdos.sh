#!/bin/bash

#SBATCH -A YourAccount
#SBATCH -J bfs-erdos-cca
#SBATCH -p general
#SBATCH -o erdos-bfs_%j.txt
#SBATCH -e erdos-bfs_%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=YourEmail
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=128
#SBATCH --time=00:30:00
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

CHIP_DIM_VALUES=("32" "64" "128")
#CHIP_DIM_VALUES=("16")
#CHIP_DIM_VALUES=("$1")

declare -A MEMORY_CC
declare -A VICINITY
declare -A MAX_EDGES

MEMORY_CC[16]=1148000
VICINITY[16]=1
MAX_EDGES[16]=20

MEMORY_CC[32]=512000
VICINITY[32]=1
MAX_EDGES[32]=20

MEMORY_CC[64]=328000
VICINITY[64]=1
MAX_EDGES[64]=20

MEMORY_CC[128]=52000
VICINITY[128]=1
#VICINITY[128]=128
MAX_EDGES[128]=20

MIN_EDGES=20

declare -A RHIZOME_CUTOFF

# Not needed here
RHIZOME_CUTOFF[1]=431800

RHIZOME_CUTOFF[2]=215900

RHIZOME_CUTOFF[4]=107950

RHIZOME_CUTOFF[8]=53975

RHIZOME_CUTOFF[16]=26987

# RHIZOMES=("16" "1" "2" "4" "8")
RHIZOMES=("1")

for CHIP_DIM in "${CHIP_DIM_VALUES[@]}"; do
    memory=${MEMORY_CC[$CHIP_DIM]}
    vicinity=${VICINITY[$CHIP_DIM]}
    max_edges=${MAX_EDGES[$CHIP_DIM]}
    max_edges=20
 
    vicinity=${CHIP_DIM}

    for RHIZOME in "${RHIZOMES[@]}"; do
	CUTOFF=${RHIZOME_CUTOFF[$RHIZOME]}
            srun -N 1 -n 1 -c ${threads} ${exe} -r ${repository} -d ${dataset} \
                 -cc cc -cxx c++ -vicinity ${vicinity} \
                 -minedge ${MIN_EDGES} -maxedge ${max_edges} \
                 -chip_dim ${CHIP_DIM} \
                 -memory_cc ${memory} -rhizomes ${RHIZOME} -cutoff ${CUTOFF} -script run_bfs_erdos.zsh
    done
done

