import sys
import glob
import re

import subprocess

scripts_path = (
  "PATH-TO/CCA-Simulator/Papers/ICPP_2024/Scripts"
)
reader_script = scripts_path + "/read_cca_simulator_output.py"


def get_matching_file_names(patterns):
    matching_files = []
    for pattern in patterns:
        matching_files.extend(glob.glob(pattern))
    return matching_files


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python script.py <output_file_name> <file_patterns> ")
        sys.exit(1)

    consolidated_results_output_file = sys.argv[1]
    file_patterns = sys.argv[2:]

    matching_files = get_matching_file_names(file_patterns)

    if not matching_files:
        print("No matching files found.")
    else:
        # print("Matching file names:")
        print("consolidated_results_output_file: ", consolidated_results_output_file)
        # Write the consolidated results header in consolidated_results_output_file
        with open(consolidated_results_output_file, "w") as file:
            file.write(
                "trail,network,termination,throttle,recv_buff_size,total_cycles,actions_created,actions_performed,actions_false_pred,diffusions_created,diffusions_performed,diffusions_false_pred,avg_cells_active_percent,diffusions_filtered,actions_overlaped,diffusions_pruned,operons_moved,dim_x,dim_y,min_edges,max_edges,vicinity,pruning,rpvo_max,rhizome_cuttoff,congestion_threshold_value,memory_cc,chip_area,totE,time,avg_objects_per_cc,total_objects_created,vertices,edges,graph_file,routing_algorithm\n"
            )

        print(
            "trail, network, termination, throttle, recv_buff_size, total_cycles, actions_created, actions_performed, actions_false_pred, diffusions_created,diffusions_performed,diffusions_false_pred, "
            "avg_cells_active_percent, diffusions_filtered, actions_overlaped, diffusions_pruned, operons_moved, dim_x, dim_y, min_edges, max_edges, vicinity, pruning, rpvo_max, rhizome_cuttoff, congestion_threshold_value, memory_cc, "
            "chip_area, totE, time, avg_objects_per_cc, total_objects_created, vertices, edges, graph_file, routing_algorithm"
        )

        for file_name in matching_files:
            # print(file_name)
            # Use regular expression to tokenize the input string
            tokens = re.split(r"_", file_name)
            """ print(tokens) """
            # Assign tokens to variables
            app = tokens[0]
            graph = tokens[2]
            vertices = tokens[4]
            edges = tokens[6]
            rpvo_max = tokens[8]
            rhizome_cuttoff = tokens[10]
            trail = tokens[12]
            dim_x = tokens[14]
            dim_y = tokens[16]
            throttle = tokens[20]
            recvbuff = tokens[22]
            vicinity = tokens[24]
            ghosts_max = tokens[27]
            edges_min = tokens[30]
            edges_max = tokens[33]
            termimation = tokens[35]
            network = tokens[37]
            pruning = tokens[40]

            # Print the values of the variables
            """ print("App:", app)
            print("square_x:", dim_x)
            print("square_y:", dim_y)
            print("graph:", graph)
            print("vertices:", vertices)
            print("edges:", edges)
            print("throttle:", throttle)
            print("recvbuff:", recvbuff)
            print("vicinity:", vicinity)
            print("edges_max:", edges_max)
            print("ghosts_max:", ghosts_max)
            print("termimation:", termimation)
            print("rpvo_max:", rpvo_max)
            print("network:", network)
            print("pruning:", pruning) """

            reader_to_call = [
                "python3",
                reader_script,
                file_name,
                trail,
                termimation,
                vicinity,
                ghosts_max,
                edges_min,
                edges_max,
                network,
                pruning,
                rpvo_max,
                rhizome_cuttoff,
                consolidated_results_output_file,
            ]

            # Run the script as a separate process with arguments
            subprocess.call(reader_to_call)
