import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 12,
    'axes.labelsize': 12,
    'legend.fontsize': 10,
    'lines.linewidth': 1.5,
    'axes.grid': True,
    'grid.alpha': 0.3
})

def clean_data(df, time_col='t'):
    df = df.sort_values(by=time_col).reset_index(drop=True)
    return df[df[time_col] > 0]

def main():
    runs = ['run_1', 'run_2', 'run_3']
    noise_labels = ['Noise 0m', 'Noise 5m', 'Noise 10m']
    colors = ['blue', 'orange', 'red']
    
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle('Navigation Performance Comparison (0m, 5m, 10m GPS Noise)', fontsize=16, fontweight='bold')
    
    rho_s = 10.67 / 2  # Veiligheidsmarge
    
    for i, run_dir in enumerate(runs):
        output_dir = f'output/{run_dir}'
        try:
            df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')), 't')
            df_other2 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other2.csv')), 't')
        except FileNotFoundError:
            print(f"Skipping {run_dir}: Own.csv of Other2.csv niet gevonden.")
            continue
            
        # --- Plot 1: Distance to Vessel (Safety Margin) ---
        min_len = min(len(df_own), len(df_other2))
        dist = np.sqrt((df_own['x_true'].iloc[:min_len] - df_other2['x_true'].iloc[:min_len])**2 + 
                       (df_own['y_true'].iloc[:min_len] - df_other2['y_true'].iloc[:min_len])**2)
        axs[0].plot(df_own['t'].iloc[:min_len], dist - rho_s, label=noise_labels[i], color=colors[i])
        
        # --- Plot 2: Heading Tracking ---
        axs[1].plot(df_own['t'], np.degrees(df_own['psi']), label=f'Actual ({noise_labels[i]})', color=colors[i])
        if i == 0:
            axs[1].plot(df_own['t'], np.degrees(df_own['psi_d']), label='Reference', color='black', linestyle='--')
            
        # --- Plot 3: Path Planning Accuracy (Tracking Deviation in Y) ---
        axs[2].plot(df_own['t'], df_own['y_true'], label=f'Actual Y ({noise_labels[i]})', color=colors[i])

    axs[0].set_title('Safety Margin (Distance to Target 2)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel(r'$d - \rho_s$ (m)')
    axs[0].axhline(0, color='red', linestyle='--', label='Collision Boundary')
    axs[0].legend()
    
    axs[1].set_title('Heading Tracking')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Heading (degrees)')
    axs[1].legend()
    
    axs[2].set_title('Path Planning Accuracy (Y-Position)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Y Position (m)')
    axs[2].legend()

    plt.tight_layout()
    os.makedirs('output', exist_ok=True)
    plt.savefig('output/Navigation_Comparison.png', dpi=300)
    plt.show()

if __name__ == '__main__':
    main()