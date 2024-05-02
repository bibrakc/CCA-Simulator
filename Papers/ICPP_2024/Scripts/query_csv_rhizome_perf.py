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

network = "TORUS"
chips = df["dim_x"].unique()

dim_x=128
print(f"dim_x: {dim_x}")


# Filter rows based on base of rpvo_max=1 for rhizome performance
filtered_df = df[
    (df["dim_x"] == dim_x) & (df["network"] == network) & (df["rpvo_max"] == 1)
]
min_index = filtered_df["total_cycles"].idxmin()
min_row = df.loc[min_index]
total_cycles_rpvo1_value = min_row.total_cycles
print(f"total_cycles_rpvo1_value: {total_cycles_rpvo1_value}")


# Filter rows based on base of rpvo_max=4 for rhizome performance
filtered_df = df[
    (df["dim_x"] == dim_x) & (df["network"] == network) & (df["rpvo_max"] == 4)
]
min_index = filtered_df["total_cycles"].idxmin()
min_row = df.loc[min_index]
total_cycles_rpvo4_value = min_row.total_cycles
print(f"total_cycles_rpvo4_value: {total_cycles_rpvo4_value}")
speedup = total_cycles_rpvo1_value / total_cycles_rpvo4_value
print(f"speed up rpvo4: {speedup:.2f}")


# Filter rows based on base of rpvo_max=16 for rhizome performance
filtered_df = df[
    (df["dim_x"] == dim_x) & (df["network"] == network) & (df["rpvo_max"] == 16)
]
min_index = filtered_df["total_cycles"].idxmin()
min_row = df.loc[min_index]
total_cycles_rpvo16_value = min_row.total_cycles
print(f"total_cycles_rpvo16_value: {total_cycles_rpvo16_value}")
speedup = total_cycles_rpvo1_value / total_cycles_rpvo16_value
print(f"speed up rpvo16: {speedup:.2f}")


