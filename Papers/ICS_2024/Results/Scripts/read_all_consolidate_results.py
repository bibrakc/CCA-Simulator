import sys
import glob
import re

import subprocess

scripts_path = '/N/u/bchandio/BigRed200/Research/git_repos/benchmarks_CCA/Jan_5_ICS/scripts'
reader_script = scripts_path+'/read_cca_simulator_output.py'


def get_matching_file_names(patterns):
    matching_files = []
    for pattern in patterns:
        matching_files.extend(glob.glob(pattern))
    return matching_files


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python script.py <output_file_name> <file_patterns> ")
        sys.exit(1)

    file_patterns = sys.argv[2:]
    consolidated_results_output_file = sys.argv[1]
    matching_files = get_matching_file_names(file_patterns)

    if not matching_files:
        print("No matching files found.")
    else:
        # print("Matching file names:")
        print('consolidated_results_output_file: ',
              consolidated_results_output_file)
        # Write the consolidated results in consolidated_results_output_file
        with open(consolidated_results_output_file, 'w') as file:
            file.write('network,termination,throttle,recv_buff_size,total_cycles,actions_created,actions_performed,actions_false_pred,avg_cells_active_percent,operons_moved,dim_x,dim_y,max_edges,vicinity,congestion_threshold_value,total_memory,avg_objects_per_cc,total_objects_created,vertices,edges,graph_file,routing_algorithm\n')

        print('network, termination, throttle, recv_buff_size, total_cycles, actions_created, actions_performed, actions_false_pred, '
              'avg_cells_active_percent, operons_moved, dim_x, dim_y, max_edges, vicinity, congestion_threshold_value, total_memory, '
              'avg_objects_per_cc, total_objects_created, vertices, edges, graph_file, routing_algorithm')

        for file_name in matching_files:
            # print(file_name)
            # Use regular expression to tokenize the input string
            tokens = re.split(r'_', file_name)
            """ print(tokens) """
            # Assign tokens to variables
            app = tokens[0]
            shape = tokens[1]
            dim_x = tokens[3]
            dim_y = tokens[5]
            graph = tokens[7]
            vertices = tokens[9]
            edges = tokens[11]
            throttle = tokens[15]
            recvbuff = tokens[17]
            vicinity = tokens[19]
            edges_max = tokens[22]
            termimation = tokens[24]
            network = tokens[26]
            # network = "TORUS"

            # Print the values of the variables
            print("App:", app)
            print("square_x:", dim_x)
            print("square_y:", dim_y)
            print("graph:", graph)
            print("vertices:", vertices)
            print("edges:", edges)
            print("throttle:", throttle)
            print("recvbuff:", recvbuff)
            print("vicinity:", vicinity)
            print("edges_max:", edges_max)
            print("termimation:", termimation)
            print("network:", network)
	    
            reader_to_call = ['python3', reader_script,
                              file_name, termimation, vicinity, edges_max, network, consolidated_results_output_file]

            # Run the script as a separate process with arguments
            subprocess.call(reader_to_call)
