# CCA Chip Animation Tools

Visualize compute cell activity during simulation. These tools read the
animation output produced by the simulator and render each cycle as a frame,
showing which cells are active, routing messages, or idle.

## Generating animation data

Build the simulator with `-DANIMATION=true` and run any application. The output
file will contain per-cycle cell activity data.

```bash
CC=gcc-13 CXX=g++-13 cmake -S ../.. -B build -DANIMATION=true -DTHROTTLE=true
cmake --build build -j 10

./build/BFS_CCASimulator \
  -f ../../Input_Graphs/Erdos-Renyi_ef_5_v_6.edgelist \
  -g Erdos -s square -root 0 -od ./Output
```

The output directory will contain a file ending in `_active_animation` with
per-cycle activation data.

## Creating an animation

```bash
python3 cca_chip_active_status_animation.py /path/to/output/file_active_animation
```

This reads the simulation data and generates an animated GIF/video showing the
CCA chip as a grid with cells lighting up as they process actions and route
operons.

## Creating a single-frame snapshot

```bash
python3 cca_chip_active_status_snapshot.py /path/to/output/file_active_animation <cycle>
```

## Included animations

| File | Description |
|---|---|
| `BFS_32x32_v_1024_e_10240_th_ON_SH_ON.gif` | BFS on a 32×32 chip (1024 vertices, 10K edges, throttle ON) |
| `BFS_32x32_th_ON_SHON.gif` | BFS on a 32×32 chip with throttle and shuffle |
| `BFS_192x192_Dimension Ordered Horizontal First Routing.gif` | BFS on a 192×192 chip with dimension-ordered routing |
| `SSSP_TH_ON_192x192_Dimension Ordered Horizontal First Routing.gif` | SSSP with throttle on 192×192 |
| `SSSP_TH_OFF_192x192_Dimension Ordered Horizontal First Routing.gif` | SSSP without throttle on 192×192 (shows congestion) |
| `Streaming_Dynamic_BFS_32x32_v_1000_egdeSample_SH_ON.gif` | Streaming dynamic graph BFS with edge sampling |
| `Legend_Animation.png` | Color legend for cell states |

## Legend

<img src="Legend_Animation.png" alt="Legend" width="400"/>

- **Active cells** are processing actions (predicate + work) or creating diffusions
- **Routing cells** are forwarding operons toward their destination
- **Idle cells** have no queued work

## Requirements

- Python 3
- matplotlib
- numpy
