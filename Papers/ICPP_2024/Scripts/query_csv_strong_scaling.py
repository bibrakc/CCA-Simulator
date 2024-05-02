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


# Filter rows based on dim_x == 16 for base of strong scaling
filtered_df = df[
    (df["dim_x"] == 16) & (df["network"] == network) & (df["rpvo_max"] == 1)
]
min_index = filtered_df["total_cycles"].idxmin()
min_row = df.loc[min_index]
total_cycles_16_value = min_row.total_cycles
print(f"total_cycles_16_value: {total_cycles_16_value}")

# chip = [32, 64, 128]
for dim_x in chips:
    rpvo_max = 1
    if dim_x == 128 or dim_x == 64:
        rpvo_max = 16
    # Filter rows based on conditions
    filtered_df = df[
        (df["dim_x"] == dim_x)
        & (df["network"] == network)
        & (df["rpvo_max"] == rpvo_max)
    ]

    # print(filtered_df)

    min_index = filtered_df["total_cycles"].idxmin()
    min_row = df.loc[min_index]

    # print(min_row)

    total_cycles_value = min_row.total_cycles
    speedup = total_cycles_16_value / total_cycles_value
    print(f"dim_x: {dim_x}")
    print(f"total_cycles_value: {total_cycles_value}")
    print(f"speed up: {speedup:.2f}")
