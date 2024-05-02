import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import numpy as np

# Data
datasets = ["E18", "R18", "AM", "LN", "LJ", "WK", "R22"]
chip_sizes = [16, 32, 64, 128]

# Performance data
performance_data = {
    "Actions Overlapped": {
        "E18": [64.71, 78.48, 77.66, 73.97],
        "R18": [71.08, 79.57, 67.50, 64.09],
        "AM": [42.18, 58.44, 64.35, 34.30],
        "LN": [4.16, 25.80, 28.21, 16.12],
        "LJ": [75.64, 83.11, 81.85, 77.94],
        "WK": [79.07, 83.17, 81.84, 71.56],
        "R22": [78.12, 81.26, 80.03, 75.73],
    },
    "Diffusions Pruned": {
        "E18": [29.44, 29.46, 27.10, 26.01],
        "R18": [17.71, 16.33, 15.25, 13.69],
        "AM": [9.45, 9.64, 10.69, 7.88],
        "LN": [2.70, 3.97, 3.01, 1.36],
        "LJ": [27.65, 27.23, 26.19, 25.26],
        "WK": [26.11, 26.02, 25.22, 24.38],
        "R22": [20.10, 19.39, 17.49, 16.72],
    },
}

# Set the bar width
bar_width = 1
num_elements = len(datasets)
twice_bar = bar_width * 2
gap = 0.5


# Create subplots for each chip size
fig, axs = plt.subplots(1, len(chip_sizes), figsize=(14, 5), sharey=True)
# Flatten the axs array
""" axs = axs.flatten() """

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
        performance_data["Actions Overlapped"][dataset][i] for dataset in datasets
    ]
    data_energy = [
        performance_data["Diffusions Pruned"][dataset][i] for dataset in datasets
    ]

    # Create bars
    bar1 = ax.bar(
        Speedup_bar_pos,
        data_speedup,
        align="center",
        width=bar_width,
        label="Actions Overlapped",
        color="yellow",
        linewidth=1,
        edgecolor="black",
        hatch="///",
    )
    bar2 = ax.bar(
        Energy_bar_pos,
        data_energy,
        align="center",
        width=bar_width,
        label="Diffusions Pruned Increase",
        color="magenta",
        linewidth=1,
        edgecolor="black",
        hatch="--",
    )

    # ax.set_ylim(0.5, 2)

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
legends_text = ["Actions Overlapped", "Diffusions Pruned"]
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
