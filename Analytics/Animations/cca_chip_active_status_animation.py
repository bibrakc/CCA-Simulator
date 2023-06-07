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

import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

args = sys.argv

# Read input data from file
filename = args[1]

with open(filename, 'r') as file:
    # read the header line and discard it
    header = file.readline()

    # read the next line and split it into variables
    cycles, dim_x, dim_y = file.readline().strip().split()

    # convert cycles, dim_x, and dim_y to integers
    cycles, dim_x, dim_y = map(int, [cycles, dim_x, dim_y])

    # read the header line for the table and discard it
    header = file.readline()

    frames = [line.strip() for line in file]

# Parse frames and convert to numpy arrays
grid_size = (dim_x, dim_y)
num_frames = len(frames)
grid_data = np.zeros((num_frames, *grid_size))
for i, frame in enumerate(frames):
    rows = frame.split(',')
    for j, row in enumerate(rows):
        grid_data[i, j] = list(map(int, row.strip().split()))

# Create a figure and axis for the animation
fig, ax = plt.subplots(figsize=(8, 8))

# Set the parameters for the H-tree
x_htree = (dim_x / 2) - 16
y_htree = (dim_y / 2) + 15
length_htree = min(dim_x, dim_y)
depth_htree = 4

# Plot the initial grid
cmap = plt.cm.colors.ListedColormap(['black', 'red'])  # Set custom colormap
grid = ax.imshow(grid_data[0], cmap=cmap, interpolation='nearest', alpha=0.85)

# Recursive function to draw the H-tree


def draw_h_tree(x, y, length, depth):
    if depth == 0:
        return

    x0 = x - length / 4
    x1 = x + length / 4
    y0 = y - length / 2.4
    y1 = y + length / 2.4

    # Draw horizontal lines
    ax.plot([x0, x1], [y, y], '--g')

    # Draw vertical lines
    ax.plot([x0, x0], [y0, y1], '--g')
    ax.plot([x1, x1], [y0, y1], '--g')

    # Recursively draw H-trees at the corners
    new_length = length / 2
    new_depth = depth - 1
    draw_h_tree(x0, y0, new_length, new_depth)  # Bottom left
    draw_h_tree(x0, y1, new_length, new_depth)  # Top left
    draw_h_tree(x1, y0, new_length, new_depth)  # Bottom right
    draw_h_tree(x1, y1, new_length, new_depth)  # Top right

# Update function for animation


def update(frame):
    grid.set_array(grid_data[frame])
    if frame == 1:
        draw_h_tree(x_htree, y_htree, length_htree, depth_htree)
    if frame == num_frames - 1:
        ani.event_source.stop()  # Stop the animation when it reaches the last frame
    return [grid]


# Create the animation
ani = animation.FuncAnimation(
    fig, update, frames=num_frames, interval=60)  # Increase the interval

# Set the grid cell size and ticks
ax.set_xticks(np.arange(grid_size[1]))
ax.set_yticks(np.arange(grid_size[0]))
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.tick_params(length=0)

# Label the axes and title the animation
ax.set_xlabel('Columns of Compute Cells')
ax.set_ylabel('Rows of Compute Cells')
ax.set_title('CCA Chip Activation Per Compute Cell')

""" # Save the animation as a GIF file
output_filename = 'animation.gif'
ani.save(output_filename, writer='pillow', dpi=70) """

# Display the plot
plt.show()
