import matplotlib.pyplot as plt
import numpy as np

datasets = ["E18", "R18", "AM", "LN", "LJ", "WK", "R22"]
datasets_R = ["E18", "R18", "AM", "LN", "LJ", "WK", "R22", "WK-Rh", "R22-Rh"]

chip_size = ["16", "32", "64", "128"]

apps = ["BFS", "SSSP", "Page Rank"]


# Performance improvement data
performance_data = {
    "16": {
        "E18": [1.00, 1.00, 1.00],
        "R18": [1.00, 1.00, 1.00],
        "AM": [1.00, 1.00, 1.00],
        "LN": [1.00, 1.00, 1.00],
        "LJ": [1.00, 1.00, 1.00],
        "WK": [1.00, 1.00, 1.00],
        "R22": [1.00, 1.00, 1.00],
    },
    "32": {
        "E18": [1.94, 1.92, 2.63],
        "R18": [1.98, 2.01, 2.38],
        "AM": [1.94, 1.79, 2.64],
        "LN": [1.88, 1.79, 2.77],
        "LJ": [2.00, 1.74, 2.48],
        "WK": [1.98, 1.84, 2.09],
        "R22": [2.13, 2.22, 2.34],
    },
    "64": {
        "E18": [3.77, 3.98, 6.79],
        "R18": [2.63, 3.05, 5.45],
        "AM": [3.73, 3.47, 6.69],
        "LN": [3.57, 3.36, 6.53],
        "LJ": [4.26, 4.03, 6.28],
        "WK": [2.49, 1.95, 3.36],
        "R22": [4.19, 4.64, 5.19],
        "WK-Rh": [4.07, 4.41, 5.74],
        "R22-Rh": [3.94, 4.57, 5.73],
    },
    "128": {
        "E18": [7.61, 7.48, 17.15],
        "R18": [5.16, 4.57, 9.19],
        "AM": [4.44, 4.36, 12.02],
        "LN": [6.60, 5.72, 15.42],
        "LJ": [9.05, 8.55, 14.70],
        "WK": [2.73, 1.82, 3.75],
        "R22": [5.27, 5.89, 8.24],
        "WK-Rh": [6.36, 5.23, 11.33],
        "R22-Rh": [7.94, 6.32, 12.35],
    },
}

# Create subplots for each buffer size
fig, axs = plt.subplots(1, len(apps), figsize=(14, 5))#, sharey=True)


# Set the bar width
bar_width = 1
num_elements = len(datasets)
thrice_bar = bar_width * 4
gap = 0.4

# Initialize the list with the first element (gap)
chip_16_bar_pos = [0]
for _ in range(num_elements - 1):
    # Calculate the next element and add it to the list
    next_element = chip_16_bar_pos[-1] + thrice_bar + gap
    chip_16_bar_pos.append(next_element)

# Create a new positions by adding bar_width to each element of the chip_16_bar_pos
chip_32_bar_pos = [element + bar_width for element in chip_16_bar_pos]
chip_64_bar_pos = [element + bar_width for element in chip_32_bar_pos]

# chip_32_bar_pos[-1] = chip_32_bar_pos[-1] + 3

chip_64_bar_pos.append(chip_64_bar_pos[-1] + thrice_bar)
chip_64_bar_pos.append(chip_64_bar_pos[-1] + thrice_bar)
chip_128_bar_pos = [element + bar_width for element in chip_64_bar_pos]

# Loop through buffer sizes and create a subplot for each
plot_for_legend = []
for i, app in enumerate(apps):
    ax = axs[i]

    # Extract data for the current buffer size
    data_16 = [performance_data["16"][dataset][i] for dataset in datasets]
    data_32 = [performance_data["32"][dataset][i] for dataset in datasets]
    data_64 = [performance_data["64"][dataset][i] for dataset in datasets_R]
    data_128 = [performance_data["128"][dataset][i] for dataset in datasets_R]

    # Create bars
    bar1 = ax.bar(
        chip_16_bar_pos,
        data_16,
        align="center",
        width=bar_width,
        label="16",
        color="#B00002",
        linewidth=0.8,
        edgecolor="black",
        hatch="///",
    )
    bar2 = ax.bar(
        chip_32_bar_pos,
        data_32,
        align="center",
        width=bar_width,
        label="32",
        color="yellow",
        linewidth=0.8,
        edgecolor="black",
        hatch="--",
    )

    bar3 = ax.bar(
        chip_64_bar_pos,
        data_64,
        align="center",
        width=bar_width,
        label="64",
        color="#2F5597",
        linewidth=0.8,
        edgecolor="black",
        hatch="x",
    )
    bar4 = ax.bar(
        chip_128_bar_pos,
        data_128,
        align="center",
        width=bar_width,
        label="128",
        color="green",
        linewidth=0.8,
        edgecolor="black",
        hatch="*",
    )

    # Add a vertical line between the bars
    line_pos = chip_32_bar_pos[-1] + 9 * gap
    print(f"line_pos: {line_pos}")
    ax.axvline(x=line_pos, color="black", linestyle="--")

    # Set labels and ticks
    if i == 0:
        plot_for_legend.append(bar1)
        plot_for_legend.append(bar2)
        plot_for_legend.append(bar3)
        plot_for_legend.append(bar4)
    ax.set_title(f"{app}", fontsize=18)
    ax.set_ylabel("Speed Up", fontsize=14)
    ax.yaxis.set_tick_params(labelsize=14)
    ax.set_yticks([0, 4, 8, 16, 18])
    ax.set_xticks([element for element in chip_64_bar_pos])
    ax.set_xticklabels(datasets_R, rotation=90)
    ax.xaxis.set_tick_params(labelsize=14)
    ax.grid(axis="y", color="gray", linestyle="dashed", linewidth=1)


# Collect legends
legends = [plot[0] for plot in plot_for_legend]
legends_text = ["Chip: 16x16", "Chip: 32x32", "Chip: 64x64", "Chip: 128x128"]
# Create a single legend for the entire figure
fig.legend(
    legends,
    legends_text,
    loc="upper center",
    bbox_to_anchor=(0.55, 1.02),
    ncol=5,
    prop={"size": 12},
)

# Add a common title
""" plt.suptitle(
    'Performance Impact of Throttle and Buffer Size', fontsize=16) """

# Show the plot
plt.tight_layout()
plt.subplots_adjust(wspace=0.25)
plt.subplots_adjust(top=0.85)
plt.show()
