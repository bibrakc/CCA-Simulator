"""
BSD 3-Clause License

Copyright (c) 2024, Bibrak Qamar and Maciej Andrzej Brodowicz

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
import math

args = sys.argv

output_file = args[1]
trail = args[2]
termination_switch = args[3]
vicinity = args[4]
ghosts_max = args[5]
min_edges = args[6]
max_edges = args[7]
network = args[8]
pruning = args[9]
rpvo_max = args[10]
rhizome_cuttoff = args[11]

consolidated_results_output_file = args[12]

# Dimension-ordered (X-Y) routing
routing_algorithm = "XY"

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

# network,termination,throttle,recv_buff_size,total_cycles,actions_created,actions_performed,actions_false_pred,diffusions_created,diffusions_performed,diffusions_false_pred,avg_cells_active_percent,operons_moved,dim_x,dim_y,min_edges,max_edges,vicinity,pruning,congestion_threshold_value,memory_cc,avg_objects_per_cc,total_objects_created,vertices,edges,graph_file,routing_algorithm
ON_text = "ON"
OFF_text = "OFF"
throttle_text = ON_text
if throttle == 0:
    throttle_text = OFF_text

memory_cc = int(memory / (dim_x * dim_y))


######### AREA ENERGY PERFORMANCE AND COST MODEL #########

# total operon nearest-neighbor hops
op_hops = 52311459  # 75e6
# total operons
op_count = 1252526  # 1e6
# average operon payload
payload_bytes = 4
# actions executed
action_count = 110119  # 2e6
# bytes per action
action_bytes = 4
# Ruche distance (in nearest neighbor hops)
ruche_len = 3
# Ruche fraction of all operon traffic
ruche_op_hops = 60e6

# fontons per die
fn = 64
# SRAM per fonton
sram_bytes = 2048 * 1024  # 1*1024*1024
# clock [Hz]
freq = 1e9
# flit (operon) size
flit_bits = 256
# word size [bits]
word = 64


# From File
fn = dim_x
sram_bytes = memory_cc
op_hops = operons_moved
op_count = actions_created
payload_bytes = 4
action_count = actions_performed
action_bytes = 4


#############################################################################
### data from Dalorex paper
#############################################################################
# SRAM density (7nm) [byte/m^2]
sram_byte_density = 29.2e6 / 8 / 1e-6

# leakage power (32KB @7nm) [W/byte]
sram_byte_leakage = 16.9e-6 / 32768

# SRAM access cost per access [J/word]
sram_rd_energy = 5.8e-12
sram_wr_energy = 9.12e-12

# NoC: energy to send message bit over unit distance [J/m]
msg_bit_energy = 8e-12 / 32 / 1e-3

#############################################################################
### RV
#############################################################################
# SiFive: RV32E in 13,500 gates
# ZERO_RISCY: 11,600 gates 2-stage pipeline
# RISC V transistor count
rv_T = 12000 * 4

# RISC V area (vanilla core)
rv_area = 0.024e-6

#############################################################################
### transistor counts
#############################################################################
# logic density (TSMC original N7)
Tdensity = 91e12

# basic component transistor counts: flip-flop, n:1 muxes
fdreT = 12
mux2T = 8
mux3T = 2 * mux2T
mux4T = 3 * mux2T
mux6T = 5 * mux2T
mux5T = 4 * mux2T
mux7T = 6 * mux2T
mux8T = 2 * mux4T + mux2T

# from Jung (Ruche nets) paper
# plain Cartesian mesh
# fifos:
ofifoT = 5 * 2 * fdreT * flit_bits
ififoT = 1 * 4 * fdreT * flit_bits
# muxes
fifomuxT = (2 * mux2T + 2 * mux4T + mux5T) * flit_bits
meshT = fifomuxT + ofifoT + ififoT

# 2D torus supposedly 50% more expensive
t2dT = meshT * 1.5

# Ruche
ruche_ififoT = ififoT + 4 * fdreT * flit_bits
ruche_ofifoT = ofifoT
ruche_fifomuxT = (2 * mux3T + 2 * mux6T + mux7T + 2 * mux2T) * flit_bits
rucheT = ruche_fifomuxT + ruche_ofifoT + ruche_ififoT

# Ruche and 2D torus
ruchet2dT = rucheT - meshT + t2dT

# execution unit
euT = 100e3


#############################################################################
def banner(s):
    LEN = 50
    l = (LEN - len(s)) // 2
    if l <= 0:
        print("***", s)
        return
    print(l * "*", s, l * "*")


# fonton area
def farea(netT):
    memA = sram_bytes / sram_byte_density
    netA = netT / Tdensity
    euA = euT / Tdensity
    return memA + netA + euA


# cumulative operon transport energy
def net_energy(hops, avg_dist):
    return hops * avg_dist * flit_bits * msg_bit_energy


# dynamic memory energy
def mem_energy(word_reads, word_writes):
    rdE = word_reads * sram_rd_energy
    wrE = word_writes * sram_wr_energy
    return rdE + wrE


# static energy
def leakage_power():
    return fn * fn * sram_bytes * sram_byte_leakage


# mesh NoC
def mesh_cost():
    # banner("2D Mesh")
    fa = farea(meshT)
    side = math.sqrt(fa)

    """ print(f"Chip area: {fn*fn*fa*1e6:.6f} mm^2")"""
    """ print(f"Total CCs: {fn*fn}")
    print(f"Total Chip Memory: {(fn*fn*sram_bytes)/(1024*1024)} MB")  """

    netE = net_energy(op_hops, side)
    memE = mem_energy(
        op_count * payload_bytes / (word / 8), action_count * action_bytes / (word / 8)
    )
    time = cycles / freq
    totE = netE + memE + leakage_power() * time
    """ print(f"Total energy: {totE:.6f} J")
    print(f"Average power: {totE/time:.6f} W")
    print(f"Total time: {time:.6f} s") """

    chip_area = fn * fn * fa * 1e6

    return chip_area, totE, time


# 2D torus NoC
def t2d_cost():
    # banner("2D torus NoC")
    fa = farea(t2dT)
    side = math.sqrt(fa)
    op_dist = 2 * (fn - 1) * side / fn

    """ print(f"Chip area: {fn*fn*fa*1e6:.6f} mm^2")"""
    """ print(f"Total CCs: {fn*fn}")
    print(f"Total Chip Memory: {(fn*fn*sram_bytes)/(1024*1024)} MB") """

    netE = net_energy(op_hops, op_dist)
    memE = mem_energy(
        op_count * payload_bytes / (word / 8), action_count * action_bytes / (word / 8)
    )
    time = cycles / freq
    totE = netE + memE + leakage_power() * time
    """ print(f"Total energy: {totE:.6f} J")
    print(f"Average power: {totE/time:.6f} W")
    print(f"Total time: {time:.6f} s") """

    chip_area = fn * fn * fa * 1e6

    return chip_area, totE, time


# Ruche NoC
def ruche_cost():
    banner("Ruche NoC")
    fa = farea(rucheT)
    side = math.sqrt(fa)

    print(f"Chip area: {fn*fn*fa*1e6:.6f} mm^2")
    print(f"Total CCs: {fn*fn}")
    print(f"Total Chip Memory: {(fn*fn*sram_bytes)/(1024*1024)} MB")

    if ruche_op_hops > op_hops:
        print("Error: count of operon hops over Ruche links greater than total hops")
        sys.exit(1)
    meshE = net_energy(op_hops - ruche_op_hops, side)
    rucheE = net_energy(ruche_op_hops, side * ruche_len)
    netE = meshE + rucheE
    memE = mem_energy(
        op_count * payload_bytes / (word / 8), action_count * action_bytes / (word / 8)
    )
    time = cycles / freq
    totE = netE + memE + leakage_power() * time
    print(f"Total energy: {totE:.6f} J")
    print(f"Average power: {totE/time:.6f} W")
    print(f"Total time: {time:.6f} s")

    chip_area = fn * fn * fa * 1e6

    return chip_area, totE, time


# 2D torus + Ruche NoC
def ruchet2d_cost():
    banner("2D torus and Ruche NoC")
    fa = farea(ruchet2dT)
    side = math.sqrt(fa)

    print(f"Chip area: {fn*fn*fa*1e6:.6f} mm^2")
    print(f"Total CCs: {fn*fn}")
    print(f"Total Chip Memory: {(fn*fn*sram_bytes)/(1024*1024)} MB")

    if ruche_op_hops > op_hops:
        print("Error: count of operon hops over Ruche links greater than total hops")
        sys.exit(1)
    t2d_dist = 2 * (fn - 1) * side / fn
    t2dE = net_energy(op_hops - ruche_op_hops, t2d_dist)
    rucheE = net_energy(ruche_op_hops, side * ruche_len)
    netE = t2dE + rucheE
    memE = mem_energy(
        op_count * payload_bytes / (word / 8), action_count * action_bytes / (word / 8)
    )
    time = cycles / freq
    totE = netE + memE + leakage_power() * time
    print(f"Total energy: {totE:.6f} J")
    print(f"Average power: {totE/time:.6f} W")
    print(f"Total time: {time:.6f} s")

    chip_area = fn * fn * fa * 1e6

    return chip_area, totE, time


####################

if network == "MESH":
    chip_area, totE, time = mesh_cost()
elif network == "TORUS":
    chip_area, totE, time = t2d_cost()
else:
    print("Invalid Network! ERROR")
# print(chip_area, totE, time
""" print(f"Chip area: {chip_area:.6f} mm^2")
print(f"Total energy: {totE:.6f} J")
print(f"Total time: {time:.6f} s") """


print(
    trail
    + ","
    + network
    + ","
    + termination_switch
    + ","
    + throttle_text
    + ","
    + str(recv_buff_size)
    + ","
    + str(total_cycles)
    + ","
    + str(actions_created)
    + ","
    + str(actions_performed)
    + ","
    + str(actions_false_pred)
    + ","
    + str(diffusions_created)
    + ","
    + str(diffusions_performed)
    + ","
    + str(diffusions_false_pred)
    + ","
    + str(avg_cells_active_percent)
    + ","
    + str(diffusions_filtered)
    + ","
    + str(actions_overlaped)
    + ","
    + str(diffusions_pruned)
    + ","
    + str(operons_moved)
    + ","
    + str(dim_x)
    + ","
    + str(dim_y)
    + ","
    + str(min_edges)
    + ","
    + str(max_edges)
    + ","
    + str(vicinity)
    + ","
    + str(pruning)
    + ","
    + str(rpvo_max)
    + ","
    + str(rhizome_cuttoff)
    + ","
    + str(congestion_threshold_value)
    + ","
    + str(memory_cc)
    + ","
    + str(f"{chip_area:.6f}")
    + ","
    + str(f"{totE:.6f}")
    + ","
    + str(f"{time:.6f}")
    + ","
    + str(avg_objects_per_cc)
    + ","
    + str(total_objects_created)
    + ","
    + str(vertices)
    + ","
    + str(edges)
    + ","
    + graph_file
    + ","
    + routing_algorithm
)

with open(consolidated_results_output_file, "a") as file:
    file.write(
        trail
        + ","
        + network
        + ","
        + termination_switch
        + ","
        + throttle_text
        + ","
        + str(recv_buff_size)
        + ","
        + str(total_cycles)
        + ","
        + str(actions_created)
        + ","
        + str(actions_performed)
        + ","
        + str(actions_false_pred)
        + ","
        + str(diffusions_created)
        + ","
        + str(diffusions_performed)
        + ","
        + str(diffusions_false_pred)
        + ","
        + str(avg_cells_active_percent)
        + ","
        + str(diffusions_filtered)
        + ","
        + str(actions_overlaped)
        + ","
        + str(diffusions_pruned)
        + ","
        + str(operons_moved)
        + ","
        + str(dim_x)
        + ","
        + str(dim_y)
        + ","
        + str(min_edges)
        + ","
        + str(max_edges)
        + ","
        + str(vicinity)
        + ","
        + str(pruning)
        + ","
        + str(rpvo_max)
        + ","
        + str(rhizome_cuttoff)
        + ","
        + str(congestion_threshold_value)
        + ","
        + str(memory_cc)
        + ","
        + str(chip_area)
        + ","
        + str(totE)
        + ","
        + str(time)
        + ","
        + str(avg_objects_per_cc)
        + ","
        + str(total_objects_created)
        + ","
        + str(vertices)
        + ","
        + str(edges)
        + ","
        + graph_file
        + ","
        + routing_algorithm
        + "\n"
    )
