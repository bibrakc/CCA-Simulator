#!/usr/bin/env python3

# Copyright (c) 2024, Maciej Andrzej Brodowicz

import math, sys

# All physical quantities expressed in unprefixed SI units

#############################################################################
### inputs
#############################################################################

# Here operon means message.

args = sys.argv

if len(sys.argv) < 3:
    print(
        "Usage: python cca_chip_perf.py <dim (for a square chip dim^2)> <memory per cc in KB> \
                <cycles> <total operon movements> <total actions or total propagates, same thing> \
                <bytes payload per operon> <actions executed> <bytes read per action> <network type>"
    )
    sys.exit(1)

# execution cycles
cycles = 10944  # 1e5
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


# From CMD
fn = int(args[1])
sram_bytes = int(args[2]) * 1024  # Assumes in KB therefore we multiply by 1024
cycles = int(args[3])
op_hops = int(args[4])
op_count = int(args[5])
payload_bytes = int(args[6])
action_count = int(args[7])
action_bytes = int(args[8])
network = args[9]


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
    banner("2D Mesh")
    fa = farea(meshT)
    side = math.sqrt(fa)

    """ print(f"Chip area: {fn*fn*fa*1e6:.6f} mm^2")
    print(f"Total CCs: {fn*fn}")
    print(f"Total Chip Memory: {(fn*fn*sram_bytes)/(1024*1024)} MB") """

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
    banner("2D torus NoC")
    fa = farea(t2dT)
    side = math.sqrt(fa)
    op_dist = 2 * (fn - 1) * side / fn

    """ print(f"Chip area: {fn*fn*fa*1e6:.6f} mm^2")
    print(f"Total CCs: {fn*fn}")
    print(f"Total Chip Memory: {(fn*fn*sram_bytes)/(1024*1024)} MB")
 """
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


#############################################################################
if __name__ == "__main__":
    if network == "MESH":
        chip_area, totE, time = mesh_cost()
        # print(chip_area, totE, time
        print(f"Chip area: {chip_area:.6f} mm^2")
        print(f"Total energy: {totE:.6f} J")
        print(f"Total time: {time:.6f} s")
    elif network == "TORUS":
        chip_area, totE, time = t2d_cost()
        print(f"Chip area: {chip_area:.6f} mm^2")
        print(f"Total energy: {totE:.6f} J")
        print(f"Total time: {time:.6f} s")
    else:
        print("Invalid Network! ERROR")
# ruche_cost()
# ruchet2d_cost()
