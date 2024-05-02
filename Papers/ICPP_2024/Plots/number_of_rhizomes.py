import matplotlib.pyplot as plt
import matplotlib.ticker as mtick
import numpy as np

# Data
rhizomes = ['1', '4', '16']


color_bar = '#2F5597'
hatch_bar = 'x'

#speedup = [1,1.38, 1.63] # 64x64 speed up WK
#speedup = [1.00, 1.01, 0.94] # 64x64 speed up R22

color_bar = 'green'
hatch_bar = '*'

speedup = [1.00, 1.71, 2.33] # 128x128 speed up WK
#speedup = [1.00, 1.52, 1.51] # 128x128 speed up R22


bar_width = 0.7
font_size = 20
# Create a bar plot
plt.figure(figsize=(4, 3))
plt.bar(rhizomes, speedup, width=bar_width, color=color_bar,
        linewidth=2, edgecolor='black', hatch=hatch_bar)
""" plt.xlabel('Input Data Graph', fontsize=16) """
plt.ylabel('Speedup', fontsize=font_size)
plt.yticks(fontsize=font_size)  # Increase the y-axis tick label size
# Set y-axis ticks as percentages
# plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(100, decimals=0))


""" plt.title(
    f'Improvement for Different Graphs (Geometric Mean: {geometric_mean:.2f}%)', fontsize=14) """
# plt.title('Degradation for Different Graphs', fontsize=20)
# Adjust the y-axis range if needed
plt.ylim(0.5, 2.5) #max(speedup) + 0.1)
plt.grid(axis='y', color='gray', linestyle='dashed', linewidth=1)

plt.xticks(fontsize=font_size)  # Increase the x-axis tick label size
# Add data labels above the bars
""" for i, v in enumerate(speedup):
    plt.text(i, v + 5, f'{v:.2f}%', ha='center', va='bottom', fontweight='bold', fontsize=14) """

plt.tight_layout()
plt.show()
