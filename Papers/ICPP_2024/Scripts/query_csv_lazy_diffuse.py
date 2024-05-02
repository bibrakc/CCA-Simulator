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


# df['network'] = df['network'].apply(str)

""" print(df["dim_x"].unique())
print(df["dim_x"].dtype)  """
# print(df["network"].dtype)
# print(df)

network = "TORUS"
chips = df["dim_x"].unique()
#chip = [32, 64, 128]
for dim_x in chips:
    #network = "MESH"
    #dim_x = 32
    
    rpvo_max = 1
    if dim_x == 128 or dim_x == 64:
        rpvo_max = 1
    # Filter rows based on conditions
    filtered_df = df[
        (df["dim_x"] == dim_x) & (df["network"] == network) & (df["rpvo_max"] == rpvo_max)
    ]

    # print(filtered_df)

    min_index = filtered_df["total_cycles"].idxmin()
    min_row = df.loc[min_index]

    # print(min_row)

    total_cycles_value = min_row.total_cycles

    actions_created_value = min_row.actions_created
    actions_performed_value = min_row.actions_performed
    diffusions_created_value = min_row.diffusions_created
    diffusions_performed_value = min_row.diffusions_performed
    actions_overlaped_value = min_row.actions_overlaped
    diffusions_pruned_value = min_row.diffusions_pruned

    actions_overlaped_percent = 100 * actions_overlaped_value / actions_created_value
    diffusions_pruned_percent = 100 * diffusions_pruned_value / diffusions_created_value
    actions_performed_percent = 100 * actions_performed_value / actions_created_value

    print(f"dim_x: {dim_x}")
    print(f"total_cycles_value: {total_cycles_value}")
    print(f"actions_performed_percent: {actions_performed_percent:.2f}")
    print(f"actions_overlaped_percent: {actions_overlaped_percent:.2f}")
    print(f"diffusions_pruned_percent: {diffusions_pruned_percent:.2f}")
