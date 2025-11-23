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

def main():
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. LOAD AND PLOT OBSTACLES
    try:
        df_obs = pd.read_csv('obstacles.csv')
        for _, row in df_obs.iterrows():
            if row['type'] == 'cylinder':
                plot_cylinder(ax, row['p1'], row['p2'], row['p3'], row['p4'], row['p5'])
    except FileNotFoundError:
        print("obstacles.csv not found. Run the ROS node first.")

    # 2. LOAD AND PLOT PATH
    try:
        df_path = pd.read_csv('pf_path.csv')
        
        # Plot Trajectory Line
        ax.plot(df_path['x'], df_path['y'], df_path['z'], color='blue', linewidth=2, label='Potential Field Path')
        
        # Mark Start and End
        if not df_path.empty:
            ax.scatter(df_path['x'].iloc[0], df_path['y'].iloc[0], df_path['z'].iloc[0], color='green', s=100, label='Start')
            ax.scatter(df_path['x'].iloc[-1], df_path['y'].iloc[-1], df_path['z'].iloc[-1], color='purple', s=100, label='Goal Reached')

    except FileNotFoundError:
        print("pf_path.csv not found. Run the ROS node first.")

    # Settings
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Potential Fields Trajectory')
    
    ax.set_xlim(-0.4, 0.4)
    ax.set_ylim(-0.4, 0.4)
    ax.set_zlim(0.0, 0.6)
    
    # Set view to top-down for better visualization of fields
    ax.view_init(elev=90, azim=-90) 
    
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()