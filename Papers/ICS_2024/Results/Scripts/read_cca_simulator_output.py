"""
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

# For argument parsing
import sys

args = sys.argv

output_file = args[1]
termination_switch = args[2]
vicinity = args[3]
max_edges = args[4]
network = args[5]

consolidated_results_output_file = args[6]

# Dimension-ordered (X-Y) routing
routing_algorithm = "XY"

# open the file in read mode
with open(output_file, 'r') as file:

    # read the header line and discard it
    header = file.readline()

    # read the next line and split it into variables
    graph_file, vertices, edges, root_vertex = file.readline().strip().split()

    # convert vertices, edges and root_vertex to integers
    vertices, edges, root_vertex = map(int, [vertices, edges, root_vertex])

    # read the header line for the table and discard it
    header = file.readline()

    # read the next line and split it into variables
    shape, dim_x, dim_y, hx, hy, hdepth, hbandwidth_max, cells, compute_cells, sink_cells, memory, throttle, recv_buff_size = file.readline().strip().split()

    # convert dim_x, dim_y, cells, and memory to integers
    dim_x, dim_y, hx, hy, hdepth, hbandwidth_max, cells, compute_cells, sink_cells, memory, throttle, recv_buff_size = map(
        int, [dim_x, dim_y, hx, hy, hdepth, hbandwidth_max, cells, compute_cells, sink_cells, memory, throttle, recv_buff_size])

    # read the header line for the table and discard it
    header = file.readline()
    congestion_policy, congestion_threshold_value = file.readline().strip().split()

    # read the header line for the table and discard it
    header = file.readline()

    # read the next line and split it into variables
    total_cycles, total_objects_created, total_actions_created, total_actions_performed, total_actions_false_pred, operons_moved = file.readline().strip().split()

    # convert cycles, invoked, performed and false_pred to integers
    cycles, objects_created, actions_created, actions_performed, actions_false_pred, operons_moved = map(
        int, [total_cycles, total_objects_created, total_actions_created, total_actions_performed, total_actions_false_pred, operons_moved])

    # read the header line for the table and discard it
    header = file.readline()
    avg_objects_per_cc = file.readline().strip()

    # read the header line for the table and discard it
    header = file.readline()
    avg_cells_active_percent = file.readline().strip()

# termination, throttle, recv_buf_size, total_cycles, total_actions, actions_performed_work, avg_active_cells_percent, dim_x, dimy, max_edges, vicinity, congestion_threshold, memory, avg_objects_per_cc, total_objects, vertices, edges, graph_file
ON_text = "ON"
OFF_text = "OFF"
throttle_text = ON_text
if (throttle == 0):
    throttle_text = OFF_text

print(network+','+termination_switch+','+throttle_text+','+str(recv_buff_size)+','+str(total_cycles)+','+str(actions_created)+','+str(actions_performed)+','+str(actions_false_pred)+','+str(avg_cells_active_percent)+','+str(operons_moved) +
      ','+str(dim_x)+','+str(dim_y)+','+str(max_edges)+','+str(vicinity)+','+str(congestion_threshold_value)+','+str(memory)+',' + str(avg_objects_per_cc) + ','+str(total_objects_created) + ','+str(vertices)+',' + str(edges) + ',' + graph_file + ',' + routing_algorithm)

with open(consolidated_results_output_file, 'a') as file:
    file.write(network+','+termination_switch+','+throttle_text+','+str(recv_buff_size)+','+str(total_cycles)+','+str(actions_created)+','+str(actions_performed)+','+str(actions_false_pred)+','+str(avg_cells_active_percent)+','+str(operons_moved) +
               ','+str(dim_x)+','+str(dim_y)+','+str(max_edges)+','+str(vicinity)+','+str(congestion_threshold_value)+','+str(memory)+',' + str(avg_objects_per_cc) + ','+str(total_objects_created) + ','+str(vertices)+',' + str(edges) + ',' + graph_file + ',' + routing_algorithm+'\n')
