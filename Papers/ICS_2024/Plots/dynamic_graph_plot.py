import matplotlib.pyplot as plt
#import pandas as pd
from matplotlib.ticker import FuncFormatter

# Provided data
data = {
    'Iteration': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
    'Dynamic': [2229, 8867, 6119, 5326, 4673, 4395, 4852, 4397, 3907, 3418],
    'Static': [2229, 4420, 6567, 8535, 10612, 12504, 14787, 16584, 18361, 20126]
}

# Calculate the ratio of Static to Dynamic for each iteration
ratio_list = [static / dynamic for static, dynamic in zip(data['Static'], data['Dynamic'])]

# Display the result
#result = dict(zip(data['Iteration'], ratio_list))
print(ratio_list)



# Create subplots for each buffer size
fig, axs = plt.subplots(figsize=(7, 4)) #, sharey=True)

dyn_bwidth = [1,4,7,10,13,16,19,22,25,28]
stat_bwidth = [2,5,8,11,14,17,20,23,26,29]

color_blue='#2F5597'
color_red='#B00002'
color_yellow='yellow'

bar_width = 1
bar1 = plt.bar(dyn_bwidth, data['Dynamic'],  align='center',
                width=bar_width, label='Dynamic-BFS', color=color_yellow, linewidth=1, edgecolor='black', hatch='///')
bar2 = plt.bar(stat_bwidth, data['Static'],  align='center',
                width=bar_width, label='Static-BFS', color=color_red, linewidth=1, edgecolor='black', hatch='--')

# plt.bar(df['Iteration'], df['Dynamic'], label='Dynamic')
# plt.bar(df['Iteration'], df['Static'], bottom=df['Dynamic'], label='Static')

# Adding labels and title
plt.xlabel('Increment', fontsize=18)
plt.ylabel('Cycles', fontsize=18)

# Format y-axis ticks to be in thousands
def format_thousands(x, pos):
    return f'{x / 1000:.0f}K'

formatter = FuncFormatter(format_thousands)
axs.yaxis.set_major_formatter(formatter)

axs.set_yticks(range(0, 20001, 5000))

plt.yticks(fontsize=14)  # Increase the y-axis tick label size

#plt.title('Dynamic vs Static Cycles over Increments')
plt.grid(axis='y', color='gray', linestyle='dashed', linewidth=1)
plt.legend(fontsize=12)

plt.xticks([1.5,4.5,7.5,10.5,13.5,16.5,19.5,22.5,25.5,28.5],fontsize=14)
axs.set_xticklabels(['1','2','3','4','5','6','7','8','9','10'])
# Show the plot
plt.tight_layout()
plt.subplots_adjust(wspace=0.25)
plt.subplots_adjust(top=0.85)
plt.show()
