#!/usr/bin/env python3
"""Generate an animation of the robot swarm communicating.

Usage: analysis/animate.py results/<preset>/<run>.db [output.mp4]
"""
import sqlite3, sys, os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation

db_path = sys.argv[1]
out_path = sys.argv[2] if len(sys.argv) > 2 else db_path.replace(".db", ".mp4")

print(f"Loading data from {db_path}...")
conn = sqlite3.connect(db_path)
p = pd.read_sql_query("SELECT timeSec, txRx, srcIp, nodeId FROM pktTxRx WHERE txRx='rx'", conn)

# Map IPs to sender node IDs
p['src_node'] = p['srcIp'].apply(lambda ip: int(ip.split('.')[-1]) - 2)
p['dst_node'] = p['nodeId']

num_robots = max(p['src_node'].max(), p['dst_node'].max()) + 1
start_time = p['timeSec'].min()
end_time = p['timeSec'].max()

# Cap animation to 5 seconds to keep it concise and punchy
end_time = min(end_time, start_time + 5.0)

print(f"Animating {num_robots} robots from {start_time:.1f}s to {end_time:.1f}s...")

fps = 30
dt = 1.0 / fps
frames = int((end_time - start_time) / dt)

# Generate smooth random waypoint mobility for the visualizer
np.random.seed(42)
area = 50.0
speed = 1.5

positions = np.zeros((frames, num_robots, 2))
# Initialize random waypoints
curr_pos = np.random.uniform(0, area, (num_robots, 2))
dest_pos = np.random.uniform(0, area, (num_robots, 2))

for i in range(frames):
    diff = dest_pos - curr_pos
    dist = np.linalg.norm(diff, axis=1)
    
    # Move towards destination
    move = speed * dt
    reached = dist < move
    
    # Update destinations for those that reached
    dest_pos[reached] = np.random.uniform(0, area, (np.sum(reached), 2))
    
    # Move others
    direction = diff / np.maximum(dist, 1e-6)[:, None]
    curr_pos[~reached] += direction[~reached] * move
    
    positions[i] = curr_pos

# Setup plotting
fig, ax = plt.subplots(figsize=(6, 6), facecolor="#111111")
ax.set_facecolor("#111111")
ax.set_xlim(0, area)
ax.set_ylim(0, area)
ax.set_xticks([])
ax.set_yticks([])
ax.set_title("5G-NR Sidelink Mode 2 Swarm", color="white", pad=15)

# Plot elements
scatter = ax.scatter([], [], c="#00ffcc", s=50, zorder=3, edgecolors="#ffffff", linewidths=0.5)
lines = []

def init():
    scatter.set_offsets(np.empty((0, 2)))
    return scatter,

def update(frame):
    t_start = start_time + frame * dt
    t_end = t_start + dt
    
    # Clear old lines
    for line in lines:
        line.remove()
    lines.clear()
    
    # Get current positions
    pos = positions[frame]
    scatter.set_offsets(pos)
    
    # Find messages delivered in this frame
    msgs = p[(p['timeSec'] >= t_start) & (p['timeSec'] < t_end)]
    
    # Draw connections
    for _, msg in msgs.iterrows():
        src = msg['src_node']
        dst = msg['dst_node']
        line, = ax.plot([pos[src, 0], pos[dst, 0]], 
                        [pos[src, 1], pos[dst, 1]], 
                        c="#ff00ff", alpha=0.4, linewidth=1.5, zorder=2)
        lines.append(line)
        
    return [scatter] + lines

print("Rendering animation (this may take a minute)...")
ani = animation.FuncAnimation(fig, update, frames=frames, init_func=init, blit=True, interval=dt*1000)

writer = animation.FFMpegWriter(fps=fps, bitrate=2000)
try:
    ani.save(out_path, writer=writer)
    print(f"Saved animation to {out_path}")
except Exception as e:
    print(f"Could not save MP4 with FFMpeg ({e}). Falling back to GIF...")
    out_path = out_path.replace(".mp4", ".gif")
    ani.save(out_path, writer=animation.PillowWriter(fps=fps))
    print(f"Saved animation to {out_path}")
