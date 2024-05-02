import pandas as pd
import sys
import os

# Check if the command line argument for the CSV file is provided
if len(sys.argv) != 2:
    print("Usage: python script.py <csv_file>")
    sys.exit(1)

# Get the CSV file name from the command line argument
csv_file = sys.argv[1]

# Read the CSV file into a Pandas DataFrame
try:
    df = pd.read_csv(csv_file)
except FileNotFoundError:
    print(f"File not found: {csv_file}")
    sys.exit(1)

chips = df["dim_x"].unique()
print(f"chips: {chips}")
#chips = [64]
for dim_x in chips:

    rpvo_max = 1
    if dim_x == 128 or dim_x == 64:
        rpvo_max = 16

    # Filter rows based o base case of MESH conditions
    filtered_df = df[
        (df["dim_x"] == dim_x) & (df["network"] == "MESH") & (df["rpvo_max"] == rpvo_max)
    ]
    
    """ print("printing for MESH")
    print(filtered_df) """


    min_index = filtered_df["total_cycles"].idxmin()
    min_row = df.loc[min_index]

    total_cycles_MESH_value = min_row.total_cycles
    total_totE_MESH_value = min_row.totE

    # Filter rows based on TORUS conditions
    filtered_df = df[
        (df["dim_x"] == dim_x) & (df["network"] == "TORUS") & (df["rpvo_max"] == rpvo_max)
    ]

    """ print("printing for TORUS")
    print(filtered_df) """

    min_index = filtered_df["total_cycles"].idxmin()
    min_row = df.loc[min_index]

    total_cycles_TORUS_value = min_row.total_cycles
    total_totE_TORUS_value = min_row.totE

    speedup_of_torus = total_cycles_MESH_value / total_cycles_TORUS_value
    energy_increase_of_torus = total_totE_TORUS_value / total_totE_MESH_value

    time_decrease_percent = 100 * (
        (total_cycles_MESH_value - total_cycles_TORUS_value) / total_cycles_MESH_value
    )
    energy_increase_percent = 100 * (
        (total_totE_TORUS_value - total_totE_MESH_value) / total_totE_TORUS_value
    )

    print(f"dim_x: {dim_x}")
    #print(f"total_cycles_MESH_value: {total_cycles_MESH_value}")
    #print(f"total_cycles_TORUS_value: {total_cycles_TORUS_value}")
    #print(f"speedup_of_torus: {speedup_of_torus:.2f}")
    #print(f"energy_increase_of_torus: {energy_increase_of_torus:.2f}")
    print(f"time_decrease_percent: {time_decrease_percent:.2f}")
    print(f"energy_increase_percent: {energy_increase_percent:.2f}")

