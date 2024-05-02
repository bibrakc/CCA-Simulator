"""
BSD 3-Clause License

Copyright (c) 2023-2024, Bibrak Qamar

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

from matplotlib.ticker import FuncFormatter

args = sys.argv

output_file = args[1]

routing_algorithm = "Dimenssion Ordered ??"

# open the file in read mode
with open(output_file, "r") as file:

    # read the header line and discard it
    header = file.readline()

    # read the next line and split it into variables
    graph_file, vertices, edges, root_vertex = file.readline().strip().split()

    # convert vertices, edges and root_vertex to integers
    vertices, edges, root_vertex = map(int, [vertices, edges, root_vertex])

    # read the header line for the table and discard it
    header = file.readline()

    # read the next line and split it into variables
    (
        shape,
        dim_x,
        dim_y,
        hx,
        hy,
        hdepth,
        hbandwidth_max,
        cells,
        compute_cells,
        sink_cells,
        memory,
        throttle,
        recv_buff_size,
    ) = (
        file.readline().strip().split()
    )

    # convert dim_x, dim_y, cells, and memory to integers
    (
        dim_x,
        dim_y,
        hx,
        hy,
        hdepth,
        hbandwidth_max,
        cells,
        compute_cells,
        sink_cells,
        memory,
        throttle,
        recv_buff_size,
    ) = map(
        int,
        [
            dim_x,
            dim_y,
            hx,
            hy,
            hdepth,
            hbandwidth_max,
            cells,
            compute_cells,
            sink_cells,
            memory,
            throttle,
            recv_buff_size,
        ],
    )

    # read the header line for the table and discard it
    header = file.readline()
    congestion_policy, congestion_threshold_value = file.readline().strip().split()

    # read the header line for the table and discard it
    header = file.readline()
    queues_configuration, action_queue_size, diffuse_queue_size = (
        file.readline().strip().split()
    )

    # read the header line for the table and discard it
    header = file.readline()
    ghost_children_max = file.readline().strip().split()

    # read the header line for the table and discard it
    header = file.readline()

    # read the next line and split it into variables
    (
        total_cycles,
        total_objects_created,
        total_actions_created,
        total_actions_performed,
        total_actions_false_pred,
        total_diffusions_created,
        total_diffusions_performed,
        total_diffusions_false_pred,
        total_diffusions_filtered,
        total_actions_overlaped,
        total_diffusions_pruned,
        operons_moved,
    ) = (
        file.readline().strip().split()
    )

    # convert cycles, invoked, performed and false_pred to integers
    (
        cycles,
        objects_created,
        actions_created,
        actions_performed,
        actions_false_pred,
        diffusions_created,
        diffusions_performed,
        diffusions_false_pred,
        diffusions_filtered,
        actions_overlaped,
        diffusions_pruned,
        operons_moved,
    ) = map(
        int,
        [
            total_cycles,
            total_objects_created,
            total_actions_created,
            total_actions_performed,
            total_actions_false_pred,
            total_diffusions_created,
            total_diffusions_performed,
            total_diffusions_false_pred,
            total_diffusions_filtered,
            total_actions_overlaped,
            total_diffusions_pruned,
            operons_moved,
        ],
    )

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
    for i in range(0, 3):  # cycles all cycles
        line = file.readline()
        line = line.strip().split("\t")
        cycle = int(line[0])
        cells_active_percent = float(line[1])
        htree_active_percent = float(line[2])
        active_status_per_cycle.append(
            (cycle, cells_active_percent, htree_active_percent)
        )

    # read the per compute cell data
    stats = pd.read_csv(file, header=0, engine="c", delimiter="\t")

# print the values to check if they were read correctly
print(shape, dim_x, dim_y, cells, memory)
print(graph_file, vertices, edges, root_vertex)
print(
    total_cycles,
    total_objects_created,
    total_actions_created,
    total_actions_performed,
    total_actions_false_pred,
    total_diffusions_created,
    total_diffusions_performed,
    total_diffusions_false_pred,
)
print(
    "congestion_policy: ",
    congestion_policy,
    ", value: ",
    congestion_threshold_value,
)
print(
    "avg_objects_per_cc: ",
    avg_objects_per_cc,
)
print(
    "action_queue_size: ",
    action_queue_size,
    ", diffuse_queue_size: ",
    diffuse_queue_size,
)
# print(cc_id, cc_x, cc_y, created, pushed, invoked, performed, false_pred,
#      stall_logic, stall_recv, stall_send, res_usage, inactive)


# print(stats.describe())

""" selected_columns = ['max_action_queue', 'max_task_queue', 'total_task_queue']
custom_percentiles = [0.1, 0.25, 0.5, 0.75, 0.85, 0.9, 0.95]  # Specify your desired percentiles here
subset_df = stats[selected_columns]
statistics = subset_df.describe(percentiles=custom_percentiles)
print(statistics)
 """


chip_config = "Chip of " + str(dim_x) + " x " + str(dim_y) + " Cells"

print(chip_config)


# Create a function to format ticks in thousands with 'K' suffix
def thousands_formatter(x, pos):
    return f"{x/1000:.0f}K"


def congestion_charts():

    # Create a figure with subplots using gridspec_kw for shared boundaries
    fig, axes = plt.subplots(1, 4, figsize=(14, 4), sharey=True)

    # Flatten the axes array to easily iterate over subplots
    axes = axes.flatten()

    bins = 25

    # blue: '#2F5597'
    # red: '#B00002'

    sns.histplot(
        data=stats,
        x="left_send_contention_total",
        bins=bins,
        ax=axes[0],
        color="#B00002",
    )
    axes[0].set_xlabel("")  # Remove x-axis label for the second subplot
    sns.histplot(
        data=stats, x="up_send_contention_total", bins=bins, ax=axes[1], color="#B00002"
    )
    axes[1].set_xlabel("")  # Remove x-axis label for the second subplot
    sns.histplot(
        data=stats,
        x="right_send_contention_total",
        bins=bins,
        ax=axes[2],
        color="#B00002",
    )
    axes[2].set_xlabel("")  # Remove x-axis label for the second subplot
    sns.histplot(
        data=stats,
        x="down_send_contention_total",
        bins=bins,
        ax=axes[3],
        color="#B00002",
    )
    axes[3].set_xlabel("")  # Remove x-axis label for the second subplot

    font_size = 16
    font_size_axis = 18

    # Set titles for each subplot
    titles = ["West Channel", "North Channel", "East Channel", "South Channel"]
    axes[0].set_ylabel("Count of Compute Cells", fontsize=font_size, fontweight="bold")
    axes[0].tick_params(axis="y", labelsize=font_size_axis)
    for ax, title in zip(axes, titles):
        ax.set_title(title, fontsize=font_size_axis, fontweight="bold")
        ax.tick_params(axis="x", labelsize=font_size_axis, rotation=90)
        # ax.set_xlabel('Cycles Spent in Contention', fontsize=11, fontweight='bold')
        ax.grid(axis="y", color="gray", linestyle="dashed", linewidth=1)

    # Format x-axis ticks using the thousands_formatter function
    for ax in axes:
        ax.xaxis.set_major_formatter(FuncFormatter(thousands_formatter))
        ax.yaxis.set_major_formatter(FuncFormatter(thousands_formatter))
        ax.set_xlim(0, 600000)  # Set the x-axis limit
        ax.set_ylim(0, 1500)  # Set the y-axis limit

    # Adjust spacing between subplots
    plt.tight_layout()
    plt.subplots_adjust(wspace=0.1)
    plt.subplots_adjust(top=0.9)
    # plt.ylim(0.5, 9) #max(speedup) + 0.1)


def active_status_chart():

    # Convert the list to a DataFrame
    active_status_df = pd.DataFrame(
        active_status_per_cycle,
        columns=["Cycle#", "Cells_Active_Percent", "Htree_Active_Percent"],
    )

    # Create the line plot using sns.lineplot
    fig, ax = plt.subplots(figsize=(16, 10))
    sns.lineplot(
        x="Cycle#",
        y="Cells_Active_Percent",
        data=active_status_df,
        label="Cells Active Percent",
        ax=ax,
        color="orange",
    )
    if hdepth != 0:
        sns.lineplot(
            x="Cycle#",
            y="Htree_Active_Percent",
            data=active_status_df,
            label="Htree Active Percent",
            ax=ax,
            color="blue",
        )

    # Add labels and title
    ax.set_xlabel("Cycles", fontsize=16)
    ax.set_ylabel("Percent of Cells Active", fontsize=16)
    # Increase font size of x and y ticks
    ax.tick_params(axis="x", labelsize=14)
    ax.tick_params(axis="y", labelsize=14)

    throttle_text = "OFF"
    if throttle == 1:
        throttle_text = "ON"

    if hdepth != 0:
        ax.set_title(
            "Mesh + Htree, Depth: "
            + str(hdepth)
            + ", Max Bandwidth: "
            + str(hbandwidth_max)
            + ", Routing: "
            + routing_algorithm
            + ", Throttle: "
            + throttle_text
            + ", Recv_buff_size: "
            + str(recv_buff_size)
            + "\nPercentage of Cells and Htree Active per Cycle",
            fontsize=16,
        )
    else:
        ax.set_title(
            "Routing: "
            + routing_algorithm
            + ", Throttle: "
            + throttle_text
            + ", Recv_buff_size: "
            + str(recv_buff_size)
            + "\nPure Mesh Network\nPercentage of Compute Cells Active per Cycle",
            fontsize=16,
        )

    # Add a larger second title
    avg_cells_active_percent_num = float(avg_cells_active_percent)
    plt.suptitle(
        "Asynchronous BFS on a CCA Chip of "
        + str(dim_x)
        + " x "
        + str(dim_y)
        + " Cells, Average Active Cells: "
        + f"{avg_cells_active_percent_num:.3g}"
        + "%",
        fontsize=16,
        fontweight="bold",
    )


# print(matplotlib.matplotlib_fname())

# Main
congestion_charts()
# active_status_chart()

# Display the plot
plt.show()
