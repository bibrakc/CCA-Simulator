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
cd /N/u/PATH/exp_5

repository=/N/u/PATH/CCA-Simulator
dataset=/N/u/PATH/Datasets_IPDPS

# Script to execute
exe=${repository}/Papers/IPDPS_2024/Runs/BFS/Experiment_5_Scaling/experiment_5.zsh

#Define the list of number of processes
threads=128
export OMP_NUM_THREADS=${threads}


# Declare arrays to store keys and max edges and vicinity tuples as strings
graph_datasets=("erdos_18" "rmat_18" "amazon" "web_google" "language")
tuples=("17,1" "27,3" "5,1" "19,3" "20,13")



# Iterate through the arrays
for ((i = 0; i < ${#graph_datasets[@]}; i++)); do
  graph="${graph_datasets[i]}"
  tuple_string="${tuples[i]}"
  IFS=',' read -r max_edge vicinity <<< "$tuple_string"

  echo "Graph: $graph, max_edge: $max_edge"
  srun -N 1 -n 1 -c ${threads} ${exe} -r ${repository} -d ${dataset} -cc cc -cxx c++ -vicinity ${vicinity} -maxedge ${max_edge} -script run_bfs_${graph}.zsh
done

