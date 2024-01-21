#!/bin/bash

#SBATCH -A r00152
#SBATCH -J bfs-ama-cca
#SBATCH -p general
#SBATCH -o amazon_%j.txt
#SBATCH -e amazon_%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=bchandio@iu.edu
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=128
#SBATCH --time=00:20:00
#SBATCH --mem=16G

#Load any modules that your program needs
#module load modulename

#change directory to:
cd /N/u/bchandio/BigRed200/Research/git_repos/benchmarks_CCA/Jan_5_ICS/Exp_1/amazon

repository=/N/u/bchandio/BigRed200/Research/git_repos/CCA-Simulator
dataset=/N/u/bchandio/BigRed200/Research/git_repos/Datasets

# Script to execute
exe=${repository}/Papers/ICS_2024/Runs/BFS/Experiment_1_Congestion/experiment_1.zsh

#Define the list of number of processes
threads=128
export OMP_NUM_THREADS=${threads}

srun -N 1 -n 1 -c ${threads} ${exe} -r ${repository} -d ${dataset} -cc cc -cxx c++ -vicinity 1 -maxedge 5 -script run_bfs_amazon.zsh
