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

# For seaborn ploting
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib

import pandas as pd
import math

args = sys.argv

output_file = args[1]

routing_algorithm = args[2]

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

    # read the header line for the table and discard it
    header = file.readline()

    # read the per cycle active status data
    active_status_per_cycle = []  # stores the active status
    for i in range(0, cycles):  # all cycles
        line = file.readline()
        line = line.strip().split('\t')
        cycle = int(line[0])
        cells_active_percent = float(line[1])
        htree_active_percent = float(line[2])
        active_status_per_cycle.append(
            (cycle, cells_active_percent, htree_active_percent))

    # read the per compute cell data
    stats = pd.read_csv(file, header=0, engine='c',
                        delimiter='\t')

# print the values to check if they were read correctly
print(shape, dim_x, dim_y, cells, memory)
print(graph_file, vertices, edges, root_vertex)
print(total_cycles, total_objects_created, total_actions_created,
      total_actions_performed, total_actions_false_pred)
print("congestion_policy: ", congestion_policy,
      ", value: ", congestion_threshold_value)
print("avg_objects_per_cc: ", avg_objects_per_cc)
# print(cc_id, cc_x, cc_y, created, pushed, invoked, performed, false_pred,
#      stall_logic, stall_recv, stall_send, res_usage, inactive)


print(stats.describe())

chip_config = "Chip of " + \
    str(dim_x)+" x "+str(dim_y)+" Cells"

print(chip_config)


def congestion_charts():

    # Create a figure with subplots
    # fig, axes = plt.subplots(2, 4, figsize=(20, 10))

    # Create a figure with subplots using gridspec_kw for shared boundaries
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))  # , sharey=True)

    # Use GridSpec to adjust subplot spacings and add borders
    # gs = gridspec.GridSpec(2, 4, figure=fig, hspace=0.3, wspace=0.2)

    # Flatten the axes array to easily iterate over subplots
    axes = axes.flatten()

    # Plot the displot for each data in the subplots
    # for i, d in enumerate(data):
    #    sns.histplot(data=d, kde=True, ax=axes[i])

    bins = 40

    sns.histplot(data=stats, x='left_send_contention_total',
                 bins=bins, ax=axes[0])
    sns.histplot(data=stats, x='left_send_contention_max',
                 bins=bins, ax=axes[4])

    sns.histplot(data=stats, x='up_send_contention_total',
                 bins=bins, ax=axes[1])
    sns.histplot(data=stats, x='up_send_contention_max', bins=bins, ax=axes[5])

    sns.histplot(data=stats, x='right_send_contention_total',
                 bins=bins, ax=axes[2])
    sns.histplot(data=stats, x='right_send_contention_max',
                 bins=bins, ax=axes[6])

    sns.histplot(data=stats, x='down_send_contention_total',
                 bins=bins, ax=axes[3])
    sns.histplot(data=stats, x='down_send_contention_max',
                 bins=bins, ax=axes[7])

    # Set titles for each subplot
    titles = ['Left Channel Contention Total', 'Up Channel Contention Total', 'Right Channel Contention Total', 'Down Channel Contention Total',
              'Left Channel Contention Max', 'Up Channel Contention Max', 'Right Channel Contention Max', 'Down Channel Contention Max']
    for ax, title in zip(axes, titles):
        ax.set_title(title, fontsize=16)
        ax.tick_params(axis='x', labelsize=14)
        ax.tick_params(axis='y', labelsize=14)
        ax.set_ylabel('Count of Cells', fontsize=14)

    throttle_text = 'OFF'
    if throttle == 1:
        throttle_text = 'ON'

    if hdepth != 0:
        # Add a main title to the figure
        plt.suptitle(chip_config+'\nMesh + Htree, Depth: ' + str(hdepth)+', Max Bandwidth: '+str(
            hbandwidth_max)+', Routing: '+routing_algorithm+', Throttle: '+throttle_text+', Recv_buff_size: '+str(recv_buff_size)+'\nContention Per Channel Histograms with Bins = '+str(bins),
            fontsize=16, fontweight='bold')
    else:
        # Add a main title to the figure
        plt.suptitle(chip_config+'\nPure Mesh Network'+', Routing: '+routing_algorithm+', Throttle: '+throttle_text+', Recv_buff_size: '+str(recv_buff_size) + '\nContention Per Channel Histograms with Bins= '+str(bins),
                     fontsize=16, fontweight='bold')

    # Adjust spacing between subplots
    plt.tight_layout()


# Plot the histogram using Seaborn
# sns.histplot(data=stats, x='actions_invoked', kde=True)
# sns.histplot(data=stats, x='actions_false_on_predicate', kde=True)
""" sns.displot(data=stats, x='actions_performed_work', bins=30) """

""" stats['percent_cycles_inactive'] = stats['cycles_inactive'].map(
    lambda x: (x/cycles)*100)

ax1 = sns.displot(data=stats, x='percent_cycles_inactive',
                  kind="kde", bw_adjust=.4)
ax1.set(xlabel='Percentage of Cycles a CC was Inactive')

ax = sns.displot(data=stats, x='percent_cycles_inactive', stat="probability")
ax.set(xlabel='Percentage of Cycles a CC was Inactive') """

# sns.displot(data=stats, x='percent_cycles_inactive', kind="ecdf")

# sns.displot(data=stats, x='actions_performed_work', kind="kde", bw_adjust=.4)

# sns.displot(data=stats, x="actions_performed_work", kind="ecdf")


def active_status_chart():

    # Convert the list to a DataFrame
    active_status_df = pd.DataFrame(active_status_per_cycle, columns=[
                                    'Cycle#', 'Cells_Active_Percent', 'Htree_Active_Percent'])

    # Create the line plot using sns.lineplot
    fig, ax = plt.subplots(figsize=(16, 10))
    sns.lineplot(x='Cycle#', y='Cells_Active_Percent',
                 data=active_status_df, label='Cells Active Percent', ax=ax, color='orange')
    if hdepth != 0:
        sns.lineplot(x='Cycle#', y='Htree_Active_Percent',
                     data=active_status_df, label='Htree Active Percent', ax=ax, color='blue')

    # Add labels and title
    ax.set_xlabel('Cycles', fontsize=16)
    ax.set_ylabel('Percent of Cells Active', fontsize=16)
    # Increase font size of x and y ticks
    ax.tick_params(axis='x', labelsize=14)
    ax.tick_params(axis='y', labelsize=14)

    throttle_text = 'OFF'
    if throttle == 1:
        throttle_text = 'ON'

    if hdepth != 0:
        ax.set_title('Mesh + Htree, Depth: ' + str(hdepth)+', Max Bandwidth: '+str(
            hbandwidth_max)+', Routing: '+routing_algorithm+', Throttle: '+throttle_text+', Recv_buff_size: '+str(recv_buff_size)+'\nPercentage of Cells and Htree Active per Cycle', fontsize=16)
    else:
        ax.set_title('Routing: '+routing_algorithm + ', Throttle: '+throttle_text+', Recv_buff_size: '+str(recv_buff_size) +
                     '\nPure Mesh Network\nPercentage of Compute Cells Active per Cycle', fontsize=16)

    # Add a larger second title
    avg_cells_active_percent_num = float(avg_cells_active_percent)
    plt.suptitle('Asynchronous BFS on a CCA Chip of ' +
                 str(dim_x)+' x '+str(dim_y)+' Cells, Average Active Cells: ' + f"{avg_cells_active_percent_num:.3g}" + '%', fontsize=16, fontweight='bold')


# print(matplotlib.matplotlib_fname())

# Main
congestion_charts()
active_status_chart()

# Display the plot
plt.show()
