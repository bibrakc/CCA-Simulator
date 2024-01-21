import matplotlib.pyplot as plt
import numpy as np

# Data for graph_a and graph_b
# datasets = ['E16', 'E18', 'R16', 'R18', 'AM', 'LN', 'WG']
datasets = ['E18', 'R18', 'AM', 'LN', 'WG']
configurations_throttle = ['Throttle OFF',
                           'Throttle ON', 'Combined Throttle and Buffer']

buffer_sizes = [1, 4, 8]

# Performance improvement data
performance_data = {
    'Throttle OFF': {
        'E18': [1.00,		1.17,	1.31],
        'R18': [1.00,		1.10,	1.20],
        'AM': [1.00,		1.29,	1.40],
        'LN': [1.00,		1.06,	1.22],
        'WG': [1.00,		1.16,	1.31],
    },
    'Throttle ON': {
        'E18': [1.00,		1.33,	1.29],
        'R18': [1.00,		1.04,	1.29],
        'AM': [1.00,		1.20,	1.14],
        'LN': [1.00,		1.30,	1.36],
        'WG': [1.00,		1.06,	1.25],
    },
    'Combined Throttle and Buffer': {
        'E18': [2.04,	    2.73,	2.65],
        'R18': [1.37,		1.42,	1.78],
        'AM': [1.25,		1.50,	1.44],
        'LN': [1.50,		1.97,	2.06],
        'WG': [1.49,		1.59,	1.87],
    },
}

""" buffer_sizes = [1, 2, 4, 8]

# Performance improvement data
performance_data = {
    'Throttle OFF': {
        'E18': [1.00,	1.17,	1.32,	1.93],
        'R18': [1.00,	1.10,	1.15,	1.19],
        'AM': [1.00,	1.42,	1.96,	3.21],
        'LN': [1.00,	1.20,	1.42,	1.96],
        'WG': [1.00,	1.06,	1.39,	1.69],
    },
    'Throttle ON': {
        'E18': [1.00,	0.97,	0.93,	0.93],
        'R18': [1.00,	1.15,	1.11,	1.09],
        'AM': [1.00,	1.04,	0.96,	1.04],
        'LN': [1.00,	1.24,	1.26,	1.28],
        'WG': [1.00,	0.95,	0.96,	1.09],
    },
    'Combined Throttle and Buffer': {
        'E18': [5.39,	5.21,	4.99,	5.03],
        'R18': [2.79,	3.22,	3.11,	3.06],
        'AM': [3.21,	3.33,	3.09,	3.33],
        'LN': [1.07,	1.33,	1.35,	1.37],
        'WG': [3.43,	3.27,	3.30,	3.74],
    },
} """


# Create subplots for each buffer size
fig, axs = plt.subplots(1, len(buffer_sizes), figsize=(
    11, 5), sharey=True)


# Set the bar width
bar_width = 1
num_elements = len(datasets)
thrice_bar = bar_width*3
gap = 0.4

# Initialize the list with the first element (gap)
throttle_OFF_bar_pos = [0]
for _ in range(num_elements - 1):
    # Calculate the next element and add it to the list
    next_element = throttle_OFF_bar_pos[-1] + thrice_bar + gap
    throttle_OFF_bar_pos.append(next_element)

# Create a new positions by adding bar_width to each element of the throttle_OFF_bar_pos
throttle_ON_bar_pos = [element + bar_width for element in throttle_OFF_bar_pos]
throttle_ON_combined_bar_pos = [
    element + bar_width for element in throttle_ON_bar_pos]

""" print(throttle_OFF_bar_pos)
print(throttle_ON_bar_pos) """

""" index = np.arange(len(datasets))
print(index)
print(index + bar_width / 2) """
# Loop through buffer sizes and create a subplot for each
plot_for_legend = []
for i, buffer_size in enumerate(buffer_sizes):
    ax = axs[i]

    # Extract data for the current buffer size
    data_off = [performance_data['Throttle OFF'][dataset][i]
                for dataset in datasets]
    data_on = [performance_data['Throttle ON'][dataset][i]
               for dataset in datasets]
    data_combined = [performance_data['Combined Throttle and Buffer'][dataset][i]
                     for dataset in datasets]

    # Create bars
    bar1 = ax.bar(throttle_OFF_bar_pos, data_off,  align='center',
                  width=bar_width, label='Throttle OFF', color='#B00002', linewidth=0.8, edgecolor='black', hatch='///')
    bar2 = ax.bar(throttle_ON_bar_pos, data_on,  align='center',
                  width=bar_width, label='Throttle ON', color='yellow', linewidth=0.8, edgecolor='black', hatch='--')
    bar3 = ax.bar(throttle_ON_combined_bar_pos, data_combined,  align='center',
                  width=bar_width, label='Combined Throttle and Buffer', color='#2F5597', linewidth=0.8, edgecolor='black', hatch='x')

    ax.set_ylim(0.5, 2.8)

    # Set labels and ticks
    if i == 0:
        ax.set_ylabel('Speed Up', fontsize=20)
        ax.yaxis.set_tick_params(labelsize=20)
        plot_for_legend.append(bar1)
        plot_for_legend.append(bar2)
        plot_for_legend.append(bar3)
    ax.set_title(f'Buffers: {buffer_size}', fontsize=20)
    ax.set_xticks([element + 2*gap for element in throttle_OFF_bar_pos])
    ax.set_xticklabels(datasets)
    ax.xaxis.set_tick_params(labelsize=20)
    ax.grid(axis='y', color='gray', linestyle='dashed', linewidth=1)


# Collect legends
legends = [plot[0] for plot in plot_for_legend]
legends_text = ['Th. Off', 'Th. On', 'Combined Buffer & Th. On']
# Create a single legend for the entire figure
fig.legend(legends, legends_text, loc='upper center',
           bbox_to_anchor=(0.55, 1.02), ncol=3,  prop={'size': 16})

# Add a common title
""" plt.suptitle(
    'Performance Impact of Throttle and Buffer Size', fontsize=16) """

# Show the plot
plt.tight_layout()
plt.subplots_adjust(wspace=0)
plt.subplots_adjust(top=0.85)
plt.show()
