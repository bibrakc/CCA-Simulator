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

# Get unique values of the 'graph_file' column
unique_graph_files = df['graph_file'].unique()

# Iterate through unique 'graph_file' values and filter the DataFrame
for graph_file_condition in unique_graph_files:
    # Filter rows based on conditions
    filtered_df = df[(df['termination'] == 'OFF') & (
        df['graph_file'] == graph_file_condition)]

    # Print the filtered DataFrame
    print(f"Filtered DataFrame for graph_file: {graph_file_condition}")
    print(filtered_df)

    # Fetch particular columns for the filtered rows
    selected_columns = ['dim_x', 'recv_buff_size', 'total_cycles', 'avg_cells_active_percent', 'operons_moved', 'max_edges', 'vicinity', 'actions_created', 'actions_performed', 'actions_false_pred',
                         'avg_objects_per_cc', 'total_objects_created', 'graph_file', 'congestion_threshold_value', 'termination']
    result = filtered_df[selected_columns]
    # Sort the result by 'dim_x'
    result = result.sort_values(by='dim_x')

    # Print the result
    print(result)
    print("\n")  # Add a separator between iterations

    # Refine the filename by removing all text before the last '/'
    refined_filename = os.path.basename(graph_file_condition)

    # Write the sorted result to a CSV file
    result.to_csv(f"{refined_filename}_strong_scaling.csv", index=False)
