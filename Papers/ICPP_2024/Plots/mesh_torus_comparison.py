import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import numpy as np

# Data
datasets = ["E18", "R18", "AM", "LN", "LJ", "WK-Rh", "R22-Rh"]
chip_sizes = [16, 32, 64, 128]

# Performance data
performance_data = {
    "Torus Time Reduction": {
        "E18": [51.67, 50.44, 45.67, 42.76],
        "R18": [43.06, 49.55, 35.68, 37.93],
        "AM": [58.06, 57.36, 55.37, 36.60],
        "LN": [36.87, 44.15, 40.16, 43.43],
        "LJ": [42.03, 44.86, 47.20, 48.08],
        "WK-Rh": [43.99, 46.09, 50.91, 44.36],
        "R22-Rh": [41.33, 47.05, 45.20, 60.27],
    },
    "Torus Energy Increase": {
        "E18": [18.10, 23.56, 24.18, 26.41],
        "R18": [26.77, 30.36, 33.38, 32.25],
        "AM": [-0.09, 10.79, 16.04, 32.43],
        "LN": [21.17, 25.73, 28.52, 31.50],
        "LJ": [25.17, 28.50, 29.53, 32.24],
        "WK-Rh": [24.66, 31.41, 32.08, 34.01],
        "R22-Rh": [24.70, 29.45, 27.94, 26.23],
    },
}


# Set the bar width
bar_width = 1
num_elements = len(datasets)
twice_bar = bar_width * 2
gap = 0.5


# Create subplots for each chip size
fig, axs = plt.subplots(1, len(chip_sizes), figsize=(14, 5.5), sharey=True)


# Initialize the list with the first element (gap)
Speedup_bar_pos = [0]
for _ in range(num_elements - 1):
    # Calculate the next element and add it to the list
    next_element = Speedup_bar_pos[-1] + twice_bar + gap
    Speedup_bar_pos.append(next_element)


# Create a new positions by adding bar_width to each element of the Speedup_bar_pos
Energy_bar_pos = [element + bar_width for element in Speedup_bar_pos]

font_size = 24

# Loop through chip sizes and create a subplot for each
plot_for_legend = []
for i, chip_size in enumerate(chip_sizes):
    ax = axs[i]

    # Extract data for the current chip size
    data_speedup = [
        performance_data["Torus Time Reduction"][dataset][i] for dataset in datasets
    ]
    data_energy = [
        performance_data["Torus Energy Increase"][dataset][i] for dataset in datasets
    ]

    # Create bars
    bar1 = ax.bar(
        Speedup_bar_pos,
        data_speedup,
        align="center",
        width=bar_width,
        label="Torus Time Reduction",
        color="deepskyblue",
        linewidth=1,
        edgecolor="black",
        hatch="///",
    )
    bar2 = ax.bar(
        Energy_bar_pos,
        data_energy,
        align="center",
        width=bar_width,
        label="Torus Energy Increase",
        color="orangered",
        linewidth=1,
        edgecolor="black",
        hatch="--",
    )

    ax.set_ylim(0, 70)

    # Set labels and ticks
    if i == 0:
        ax.set_ylabel("Percentage", fontsize=font_size)
        ax.yaxis.set_tick_params(labelsize=font_size)
        plot_for_legend.append(bar1)
        plot_for_legend.append(bar2)
        ax.yaxis.set_major_formatter(mtick.PercentFormatter(100, decimals=0))
    ax.set_title(f"Chip: {chip_size} x {chip_size}", fontsize=font_size)
    ax.set_xticks([element + gap for element in Speedup_bar_pos])
    ax.set_xticklabels(datasets, rotation=90)
    ax.xaxis.set_tick_params(labelsize=font_size)
    ax.grid(axis="y", color="gray", linestyle="dashed", linewidth=1)

# Collect legends
legends = [plot[0] for plot in plot_for_legend]
legends_text = ["Torus Time-to-Solution Decrease", "Torus Energy Increase"]
# Create a single legend for the entire figure
fig.legend(
    legends,
    legends_text,
    loc="upper center",
    bbox_to_anchor=(0.55, 1.02),
    ncol=3,
    prop={"size": 16},
)

# Add a common title
""" plt.suptitle(
    'Performance Impact of Throttle and Buffer Size', fontsize=16) """

# Show the plot
plt.tight_layout()
plt.subplots_adjust(wspace=0)
plt.subplots_adjust(top=0.85)
plt.show()
