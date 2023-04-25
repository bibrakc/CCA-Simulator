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
    shape, dim_x, dim_y, cells, memory = file.readline().strip().split()

    # convert dim_x, dim_y, cells and memory to integers
    dim_x, dim_y, cells, memory = map(int, [dim_x, dim_y, cells, memory])

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

    stats = pd.read_csv(file, header=0, engine='c', delimiter='\t')

# print the values to check if they were read correctly
print(shape, dim_x, dim_y, cells, memory)
print(graph_file, vertices, edges, root_vertex)
print(total_cycles, total_actions_invoked,
      total_actions_performed, total_actions_false_pred)
#print(cc_id, cc_x, cc_y, created, pushed, invoked, performed, false_pred,
#      stall_logic, stall_recv, stall_send, res_usage, inactive)


#print(stats.describe())

# Plot the histogram using Seaborn
#sns.histplot(data=stats, x='actions_invoked', kde=True)
#sns.histplot(data=stats, x='actions_false_on_predicate', kde=True)
#sns.displot(data=stats, x='actions_performed_work', bins=30)

stats['percent_cycles_inactive'] = stats['cycles_inactive'].map(lambda x: (x/cycles)*100)

ax1 = sns.displot(data=stats, x='percent_cycles_inactive', kind="kde", bw_adjust=.4)
ax1.set(xlabel='Percentage of Cycles a CC was Inactive')

ax = sns.displot(data=stats, x='percent_cycles_inactive', stat="probability")
ax.set(xlabel='Percentage of Cycles a CC was Inactive')

#sns.displot(data=stats, x='percent_cycles_inactive', kind="ecdf")

#sns.displot(data=stats, x='actions_performed_work', kind="kde", bw_adjust=.4)


#sns.displot(data=stats, x="actions_performed_work", kind="ecdf")


plt.show()
