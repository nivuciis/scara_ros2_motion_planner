import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def plot_cylinder(ax, x, y, z_min, z_max, radius, color='red'):
    z = np.linspace(z_min, z_max, 20)
    theta = np.linspace(0, 2*np.pi, 20)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius * np.cos(theta_grid) + x
    y_grid = radius * np.sin(theta_grid) + y
    ax.plot_surface(x_grid, y_grid, z_grid, alpha=0.3, color=color)

def plot_box(ax, min_coords, max_coords, color='blue'):
    x = [min_coords[0], max_coords[0]]
    y = [min_coords[1], max_coords[1]]
    z = [min_coords[2], max_coords[2]]
    
    # Define vertices of the cube
    vertices = [
        [[x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[0], y[1], z[0]]], # Bottom
        [[x[0], y[0], z[1]], [x[1], y[0], z[1]], [x[1], y[1], z[1]], [x[0], y[1], z[1]]], # Top
        [[x[0], y[0], z[0]], [x[0], y[1], z[0]], [x[0], y[1], z[1]], [x[0], y[0], z[1]]], # Side 1
        [[x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[1], y[1], z[1]], [x[1], y[0], z[1]]], # Side 2
        [[x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[0], z[1]], [x[0], y[0], z[1]]], # Front
        [[x[0], y[1], z[0]], [x[1], y[1], z[0]], [x[1], y[1], z[1]], [x[0], y[1], z[1]]]  # Back
    ]
    
    ax.add_collection3d(Poly3DCollection(vertices, facecolors=color, linewidths=1, edgecolors='k', alpha=0.3))

def main():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. LOAD AND PLOT OBSTACLES
    try:
        df_obs = pd.read_csv('obstacles.csv')
        for _, row in df_obs.iterrows():
            if row['type'] == 'cylinder':
                plot_cylinder(ax, row['p1'], row['p2'], row['p3'], row['p4'], row['p5'])
            elif row['type'] == 'box':
                plot_box(ax, [row['p1'], row['p2'], row['p3']], [row['p4'], row['p5'], row['p6']])
    except FileNotFoundError:
        print("obstacles.csv not found.")

    # 2. LOAD AND PLOT TREE
    try:
        df_tree = pd.read_csv('rrt_tree.csv')
        print(f"Plotting {len(df_tree)} tree branches...")
        # Plot all lines at once using a line collection would be faster, 
        # but standard plot loop is fine for <5000 nodes
        for _, row in df_tree.iterrows():
            ax.plot([row['x'], row['parent_x']], 
                    [row['y'], row['parent_y']], 
                    [row['z'], row['parent_z']], 
                    color='green', linewidth=0.3, alpha=0.5)
    except FileNotFoundError:
        print("rrt_tree.csv not found.")

    # 3. LOAD AND PLOT FINAL PATH
    try:
        df_path = pd.read_csv('rrt_path.csv')
        ax.plot(df_path['x'], df_path['y'], df_path['z'], color='black', linewidth=3, marker='o', label='Final Path')
    except FileNotFoundError:
        print("rrt_path.csv not found.")

    # Settings
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D RRT* Visualization')
    
    # Set fixed limits to match workspace
    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-0.4, 0.4)
    ax.set_zlim(0.0, 0.6)
    
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()