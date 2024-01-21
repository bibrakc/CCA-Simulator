import matplotlib.pyplot as plt
import numpy as np

# Data for graph_a and graph_b
# datasets = ['E16', 'E18', 'R16', 'R18', 'AM', 'LN', 'WG']
datasets = ['AM', 'R22', 'LJ', 'WK']
chip_size = ['16', '32', '64']
apps = ['BFS', 'SSSP']

# Performance improvement data
performance_data = {
    '16': {
        'AM': [5.12,		5.45],
        'R22': [1.58,		1.02],
        'LJ': [2.12,		2.24],
        'WK': [2.82,		2.59],
    },
    '32': {
        'AM': [2.77,		2.69],
        'R22': [1.62,		1.03],
        'LJ': [1.51,		1.49],
        'WK': [3.35,		3.55],
    },
    '64': {
        'AM': [1.60,		1.59],
        'R22': [3.60,		1.76],
        'LJ': [1.44,		1.78],
        'WK': [3.57,		3.41],
    },
}

# Create subplots for each buffer size
fig, axs = plt.subplots(1, len(apps), figsize=(
    8.5, 4)) #, sharey=True)


# Set the bar width
bar_width = 1
num_elements = len(datasets)
thrice_bar = bar_width*4
gap = 0.4

# Initialize the list with the first element (gap)
chip_16_bar_pos = [0]
for _ in range(num_elements - 1):
    # Calculate the next element and add it to the list
    next_element = chip_16_bar_pos[-1] + thrice_bar + gap
    chip_16_bar_pos.append(next_element)

# Create a new positions by adding bar_width to each element of the chip_16_bar_pos
chip_32_bar_pos = [element + bar_width for element in chip_16_bar_pos]
chip_64_bar_pos = [
    element + bar_width for element in chip_32_bar_pos]
""" chip_128_bar_pos = [
    element + bar_width for element in chip_64_bar_pos]
 """
# Loop through buffer sizes and create a subplot for each
plot_for_legend = []
for i, app in enumerate(apps):
    ax = axs[i]

    # Extract data for the current buffer size
    data_16 = [performance_data['16'][dataset][i]
                for dataset in datasets]
    data_32 = [performance_data['32'][dataset][i]
               for dataset in datasets]
    data_64 = [performance_data['64'][dataset][i]
                     for dataset in datasets]
    """ data_128 = [performance_data['128'][dataset][i]
                     for dataset in datasets] """

    # Create bars
    bar1 = ax.bar(chip_16_bar_pos, data_16,  align='center',
                  width=bar_width, label='16', color='#B00002', linewidth=0.8, edgecolor='black', hatch='///')
    bar2 = ax.bar(chip_32_bar_pos, data_32,  align='center',
                  width=bar_width, label='32', color='yellow', linewidth=0.8, edgecolor='black', hatch='--')
    bar3 = ax.bar(chip_64_bar_pos, data_64,  align='center',
                  width=bar_width, label='64', color='#2F5597', linewidth=0.8, edgecolor='black', hatch='x')
    """ bar4 = ax.bar(chip_128_bar_pos, data_128,  align='center',
                  width=bar_width, label='32', color='green', linewidth=0.8, edgecolor='black', hatch='*') """

    # Set labels and ticks
    if i == 0:  
        plot_for_legend.append(bar1)
        plot_for_legend.append(bar2)
        plot_for_legend.append(bar3)
        # plot_for_legend.append(bar4)
    ax.set_title(f'{app}', fontsize=18)
    ax.set_ylabel('Speed Up', fontsize=14)
    ax.yaxis.set_tick_params(labelsize=14)
    ax.set_xticks([element + 4*gap for element in chip_16_bar_pos])
    ax.set_xticklabels(datasets)
    ax.xaxis.set_tick_params(labelsize=14)
    ax.grid(axis='y', color='gray', linestyle='dashed', linewidth=1)


# Collect legends
legends = [plot[0] for plot in plot_for_legend]
legends_text = ['Chip: 16x16', 'Chip: 32x32', 'Chip: 64x64']
# Create a single legend for the entire figure
fig.legend(legends, legends_text, loc='upper center',
           bbox_to_anchor=(0.55, 1.02), ncol=5,  prop={'size': 12})

# Add a common title
""" plt.suptitle(
    'Performance Impact of Throttle and Buffer Size', fontsize=16) """

# Show the plot
plt.tight_layout()
plt.subplots_adjust(wspace=0.25)
plt.subplots_adjust(top=0.85)
plt.show()
