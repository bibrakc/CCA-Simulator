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
cd /N/u/PATH/exp_2

repository=/N/u/PATH/CCA-Simulator
dataset=/N/u/PATH/Datasets_IPDPS

# Script to execute
exe=${repository}/Papers/IPDPS_2024/Runs/BFS/Experiment_2_Allocation/experiment_2.zsh

#Define the list of number of processes
threads=128
export OMP_NUM_THREADS=${threads}

# Declare an associative array to store graph name and its max_edges vertex
declare -A graph_datasets

graph_datasets["erdos_18"]=17
graph_datasets["rmat_18"]=27
graph_datasets["amazon"]=5
graph_datasets["web_google"]=19
graph_datasets["language"]=20

# Iterate through all the graphs
for graph in "${!graph_datasets[@]}"; do
  max_edge="${graph_datasets[$graph]}"
  echo "Graph: $graph, max_edge: $max_edge"
  srun -N 1 -n 1 -c ${threads} ${exe} -r ${repository} -d ${dataset} -cc cc -cxx c++ -vicinity 128 -maxedge ${max_edge} -script run_bfs_${graph}.zsh
done
