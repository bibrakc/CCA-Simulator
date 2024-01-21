import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import numpy as np

# Data
graph_labels = ['LN', 'AM', 'E18', 'R18', 'WG', 'LJ', 'WK', 'R22']
vicinity_allocator_improvement_values = [33.25, 0.00, 1.70, 20.80, 16.58, 3.88, 9.57, 39.55] # in percentages

# Calculate geometric mean
""" geometric_mean = np.exp(
    np.mean(np.log(np.array(vicinity_allocator_improvement_values) / 100))) * 100 """


bar_width = 0.5
# Create a bar plot
plt.figure(figsize=(8, 4))
plt.bar(graph_labels, vicinity_allocator_improvement_values, width=bar_width, color='#2F5597',
        linewidth=2, edgecolor='black', hatch='///')
""" plt.xlabel('Input Data Graph', fontsize=16) """
plt.ylabel('Improvement', fontsize=18)
plt.yticks(fontsize=14)  # Increase the y-axis tick label size
# Set y-axis ticks as percentages
plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(100, decimals=0))


""" plt.title(
    f'Improvement for Different Graphs (Geometric Mean: {geometric_mean:.2f}%)', fontsize=14) """
# plt.title('Degradation for Different Graphs', fontsize=20)
# Adjust the y-axis range if needed
plt.ylim(0, max(vicinity_allocator_improvement_values) + 20)
plt.grid(axis='y', color='gray', linestyle='dashed', linewidth=1)

plt.xticks(fontsize=14)  # Increase the x-axis tick label size
# Add data labels above the bars
for i, v in enumerate(vicinity_allocator_improvement_values):
    plt.text(i, v + 5, f'{v:.2f}%', ha='center', va='bottom', fontweight='bold', fontsize=14)

plt.tight_layout()
plt.show()
