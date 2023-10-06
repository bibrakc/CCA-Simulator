#!/bin/bash

#SBATCH -A acount?
#SBATCH -J bfs-cca
#SBATCH -p general
#SBATCH -o exp_1_congestion_%j.txt
#SBATCH -e exp_1_congestion_%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=YOURMAIL@iu.edu
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=128
#SBATCH --time=02:30:00
#SBATCH --mem=6G

#Load any modules that your program needs
#module load modulename

#change directory to:
cd /N/u/PATH/exp_1/amazon

repository=/N/u/PATH/CCA-Simulator
dataset=/N/u/PATH/Datasets_IPDPS

# Script to execute
exe=${repository}/Papers/IPDPS_2024/Runs/BFS/Experiment_1_Congestion/experiment_1.zsh

#Define the list of number of processes
threads=128
export OMP_NUM_THREADS=${threads}

srun -N 1 -n 1 -c ${threads} ${exe} -r ${repository} -d ${dataset} -cc cc -cxx c++ -vicinity 1 -maxedge 5 -script run_bfs_amazon.zsh
