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

import pandas as pd
import math

args = sys.argv

output_file = args[1]

# open the file in read mode
with open(output_file, 'r') as file:

    # read the header line and discard it
    header = file.readline()

    # read the next line and split it into variables
    shape, dim_x, dim_y, hx, hy, hdepth, hbandwidth_max, cells, memory = file.readline().strip().split()

    # convert dim_x, dim_y, cells, and memory to integers
    dim_x, dim_y, hx, hy, hdepth, hbandwidth_max, cells, memory = map(
        int, [dim_x, dim_y, hx, hy, hdepth, hbandwidth_max, cells, memory])

    # read the header line for the table and discard it
    header = file.readline()

    # read the next line and split it into variables
    graph_file, vertices, edges, root_vertex = file.readline().strip().split()

    # convert vertices, edges and root_vertex to integers
    vertices, edges, root_vertex = map(int, [vertices, edges, root_vertex])

    # read the header line for the table and discard it
    header = file.readline()

    # read the next line and split it into variables
    total_cycles, total_actions_invoked, total_actions_performed, total_actions_false_pred = file.readline().strip().split()

    # convert cycles, invoked, performed and false_pred to integers
    cycles, invoked, performed, false_pred = map(
        int, [total_cycles, total_actions_invoked, total_actions_performed, total_actions_false_pred])

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
print(total_cycles, total_actions_invoked,
      total_actions_performed, total_actions_false_pred)
# print(cc_id, cc_x, cc_y, created, pushed, invoked, performed, false_pred,
#      stall_logic, stall_recv, stall_send, res_usage, inactive)


# print(stats.describe())

# Plot the histogram using Seaborn
# sns.histplot(data=stats, x='actions_invoked', kde=True)
# sns.histplot(data=stats, x='actions_false_on_predicate', kde=True)
sns.displot(data=stats, x='actions_performed_work', bins=30)

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


# Convert the list to a DataFrame
active_status_df = pd.DataFrame(active_status_per_cycle, columns=[
                                'Cycle#', 'Cells_Active_Percent', 'Htree_Active_Percent'])

# Create the line plot using sns.lineplot
fig, ax = plt.subplots()
sns.lineplot(x='Cycle#', y='Cells_Active_Percent',
             data=active_status_df, label='Cells Active Percent', ax=ax, color='orange')
if hdepth != 0:
    sns.lineplot(x='Cycle#', y='Htree_Active_Percent',
                 data=active_status_df, label='Htree Active Percent', ax=ax, color='blue')

# Add labels and title
ax.set_xlabel('Cycle')
ax.set_ylabel('Percent')
if hdepth != 0:
    ax.set_title('Percentage of Cells and Htree Active per Cycle')
else:
    ax.set_title('Percentage of Compute Cells Active per Cycle')
# Display the plot
plt.show()
