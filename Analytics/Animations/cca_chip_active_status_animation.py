""" 
BSD 3-Clause License

Copyright (c) 2023, Bibrak Qamar

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

from matplotlib.legend_handler import HandlerTuple
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import matplotlib.patches as mpatches

args = sys.argv

# Read input data from file
# Check if arguments are provided
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    print("No filename provided.")
    sys.exit(1)

# Set default values
show_last_frames = 0
skip_frames = 1

# Check if optional arguments are provided
if len(sys.argv) > 3:
    show_last_frames = int(sys.argv[2])
    skip_frames = int(sys.argv[3])

print("Filename:", filename)
print("Show Last Frames:", show_last_frames)
print("Skip Frames:", skip_frames)


with open(filename, "r") as file:
    # read the header line and discard it
    header = file.readline()

    # read the next line and split it into variables
    cycles, dim_x, dim_y = file.readline().strip().split()

    # convert cycles, dim_x, and dim_y to integers
    cycles, dim_x, dim_y = map(int, [cycles, dim_x, dim_y])

    # read the header line for the table and discard it
    header = file.readline()

    frames = [line.strip() for line in file]

# Mapping of scalar values to RGB colors
scalar_to_rgb = {
    0: (0, 0, 0),  # black
    1: (0, 255, 255),  # cyan
    2: (0, 255, 0),  # green
    3: (255, 255, 255),  # white
    4: (255, 255, 214),  # yellow level 1
    5: (255, 255, 163),  # yellow level 2
    6: (255, 255, 61),  # yellow level 3
    7: (255, 255, 0),  # yellow level 4 pure
    8: (255, 204, 214),  # red level 1
    9: (255, 153, 163),  # red level 2
    10: (255, 51, 61),  # red level 3
    11: (255, 0, 0),  # red level 4 pure
}


# Parse frames and convert to numpy arrays
grid_size = (dim_x, dim_y)
num_frames = len(frames)
grid_data = np.zeros((num_frames, *grid_size, 3), dtype=np.uint8)

for i, frame in enumerate(frames):
    rows = frame.split(",")
    for j, row in enumerate(rows):
        scalar_values = list(map(int, row.strip().split()))
        for k, scalar_value in enumerate(scalar_values):
            rgb_value = scalar_to_rgb[scalar_value]
            grid_data[i, j, k] = rgb_value


# Create a figure and axis for the animation
fig, ax = plt.subplots(figsize=(18, 10))

# Set the parameters for the H-tree
x_htree = (dim_x / 2) - 16.5
y_htree = (dim_y / 2) + 15
length_htree = min(dim_x, dim_y)
depth_htree = 4

htree_draw = False

# Display the RGB image
grid = ax.imshow(grid_data[0], alpha=0.80)

colors = ["black", "cyan", "green", "white", "yellow", "red"]
# Create custom legend with color-value mappings
legend_labels = {
    0: "Inactive",
    1: "Only Communicating",
    2: "Only Computing",
    3: "Computing & Communicating",
    4: "Congested Communicating",
    5: "Congested Computing & Communicating",
}
legend_elements = [
    mpatches.Circle((0, 0), radius=0.2, color=color, label=label)
    for value, label in legend_labels.items()
    for i, color in enumerate(colors)
    if i == value
]

# Add the legend to the plot
# ax.legend(handles=legend_elements, loc='upper right')
# Add the legend outside the main figure
# ax.legend(handles=legend_elements, bbox_to_anchor=(0.8, 1), loc="lower center")
# ax.legend(handles=legend_elements, bbox_to_anchor=(1.45, 1), loc='upper right')
""" ax.legend(handles=legend_elements, loc='best',
          bbox_to_anchor=(0.13, -0.525, 0.5, 0.5)) """


# Place the legend above the plot
# ax.legend(handles=legend_elements, bbox_to_anchor=(0.5, 1.21), loc='upper center', handler_map={tuple: HandlerTuple(ndivide=None)})
ax.legend(
    handles=legend_elements, bbox_to_anchor=(1.65, 1), loc="upper right", fontsize=14
)


# Recursive function to draw the H-tree
def draw_h_tree(x, y, length, depth):
    if depth == 0:
        return

    x0 = x - length / 4
    x1 = x + length / 4
    y0 = y - length / 2.4
    y1 = y + length / 2.4

    # Draw horizontal lines
    ax.plot([x0, x1], [y, y], "--g")

    # Draw vertical lines
    ax.plot([x0, x0], [y0, y1], "--g")
    ax.plot([x1, x1], [y0, y1], "--g")

    # Recursively draw H-trees at the corners
    new_length = length / 2
    new_depth = depth - 1
    draw_h_tree(x0, y0, new_length, new_depth)  # Bottom left
    draw_h_tree(x0, y1, new_length, new_depth)  # Top left
    draw_h_tree(x1, y0, new_length, new_depth)  # Bottom right
    draw_h_tree(x1, y1, new_length, new_depth)  # Top right


# For larger simulation we want to see last frames
if show_last_frames != 0:
    start_from = cycles - show_last_frames
else:
    start_from = 0

# Update function for animation


def update(frame):
    grid.set_array(grid_data[frame + start_from])
    # Set the title for each frame
    ax.set_title(
        "CCA Chip Activation Per Compute Cell - Cycle # {}".format(frame + start_from),
        fontsize=14,
    )

    if frame == 1 and htree_draw == True:
        draw_h_tree(x_htree, y_htree, length_htree, depth_htree)
    if frame == num_frames - 1:
        ani.event_source.stop()  # Stop the animation when it reaches the last frame
    return [grid]


frames_to_show = cycles - start_from
# Create the animation
ani = animation.FuncAnimation(
    fig, update, frames=range(0, frames_to_show, skip_frames), interval=70
)  # Increase the interval

# Set the grid cell size and ticks
ax.set_xticks(np.arange(grid_size[1]))
ax.set_yticks(np.arange(grid_size[0]))
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.tick_params(length=0)

# Label the axes and title the animation
ax.set_xlabel("Columns of Compute Cells", fontsize=16)
ax.set_ylabel("Rows of Compute Cells", fontsize=16)
# Add a larger second title
routing_algorithm = "Dimension Ordered Horizontal First Routing"
graph_size = "Random Directed Graph V=36K and E=0.66M"
plt.suptitle(
    "Asynchronous SSSP on a CCA Chip of "
    + str(dim_x)
    + " x "
    + str(dim_y)
    + "\n"
    + routing_algorithm
    + "\n"
    + graph_size,
    fontsize=16,
)


output_filename = (
    "SSSP_TH_OFF_" + str(dim_x) + "x" + str(dim_y) + "_" + routing_algorithm
)
# Save the animation as an MP4 file
""" ani.save(output_filename+'.mp4', writer='ffmpeg', dpi=520) """

# Save the animation as a GIF file
""" ani.save(output_filename+'.gif', writer='pillow', dpi=100) """
# ani.save(output_filename, writer='pillow', dpi=70, interval=50)

# Display the plot
plt.show()
