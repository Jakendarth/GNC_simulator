import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Apply publication-quality settings
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'legend.fontsize': 9,
    'lines.linewidth': 1.5,
    'axes.grid': True,
    'grid.alpha': 0.3
})

def clean_data(df, time_col='t'):
    """Sorteert de dataset op tijd en verwijdert de initialisatiestap (t=0)."""
    df = df.sort_values(by=time_col).reset_index(drop=True)
    return df[df[time_col] > 0]

def main():
    output_dir = 'output/run_0' 
    
    try:
        df_own = pd.read_csv(os.path.join(output_dir, 'Own.csv'))
        df_own = clean_data(df_own, 't')
        
        df_own_est = pd.read_csv(os.path.join(output_dir, 'Own_estimation.csv'))
        df_own_est = clean_data(df_own_est, 'timehat')
        
        df_other2 = pd.read_csv(os.path.join(output_dir, 'Other2.csv'))
        df_other2 = clean_data(df_other2, 't')
        
        df_other4 = pd.read_csv(os.path.join(output_dir, 'Other4.csv'))
        df_other4 = clean_data(df_other4, 't')
        
    except FileNotFoundError as e:
        print(f"Error loading files: {e}. Zorg dat run_1 succesvol is afgerond.")
        return

    os.makedirs(output_dir, exist_ok=True)

    # =========================================================================
    # FIGUUR: COMBINED ANALYSIS (2 Rijen x 3 Kolommen)
    # =========================================================================
    fig, axs = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Sensitivity Analysis: Multi-Metric Overview', fontsize=16, fontweight='bold')

    # --- Plot 1: X-Y Trajectories ---
    axs[0, 0].plot(df_own['x_true'], df_own['y_true'], label='Own Vessel', color='blue')
    axs[0, 0].plot(df_other2['x_true'], df_other2['y_true'], label='Target 2', color='green')
    axs[0, 0].plot(df_other4['x_true'], df_other4['y_true'], label='Target 4', color='purple')
    axs[0, 0].scatter(df_own['x_true'].iloc[0], df_own['y_true'].iloc[0], marker='o', color='blue', label='Start Own')
    axs[0, 0].set_xlabel('X Position (m)')
    axs[0, 0].set_ylabel('Y Position (m)')
    axs[0, 0].set_title('X-Y Trajectory')
    axs[0, 0].legend(loc='best')
    axs[0, 0].axis('equal')

    # --- Plot 2: Heading Tracking ---
    axs[0, 1].plot(df_own['t'], np.degrees(df_own['psi']), label=r'$\psi$ (Actual)', color='blue')
    axs[0, 1].plot(df_own['t'], np.degrees(df_own['psi_d']), label=r'$\psi_d$ (Reference)', color='red', linestyle='--')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Heading (degrees)')
    axs[0, 1].set_title('Heading Tracking')
    axs[0, 1].legend(loc='best')

    # --- Plot 3: Safety Margin ---
    min_len_2 = min(len(df_own), len(df_other2))
    min_len_4 = min(len(df_own), len(df_other4))
    
    dist_2 = np.sqrt((df_own['x_true'].iloc[:min_len_2] - df_other2['x_true'].iloc[:min_len_2])**2 + 
                     (df_own['y_true'].iloc[:min_len_2] - df_other2['y_true'].iloc[:min_len_2])**2)
    dist_4 = np.sqrt((df_own['x_true'].iloc[:min_len_4] - df_other4['x_true'].iloc[:min_len_4])**2 + 
                     (df_own['y_true'].iloc[:min_len_4] - df_other4['y_true'].iloc[:min_len_4])**2)
    rho_s = 10.67 / 2  
    
    axs[0, 2].plot(df_own['t'].iloc[:min_len_2], dist_2 - rho_s, color='green', label='Own vs Target 2')
    axs[0, 2].plot(df_own['t'].iloc[:min_len_4], dist_4 - rho_s, color='purple', label='Own vs Target 4')
    axs[0, 2].axhline(0, color='red', linestyle='--', label='Collision Boundary')
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel(r'$d - \rho_s$ (m)')
    axs[0, 2].set_title('Safety Margin (Vessel to Vessel)')
    axs[0, 2].legend(loc='best')

    # --- Plot 4: Rudder Angle ---
    if 'delta' in df_own.columns:
        axs[1, 0].plot(df_own['t'], np.degrees(df_own['delta']), color='black', label=r'Rudder Angle $\delta$')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Angle (degrees)')
        axs[1, 0].set_title('Control Input: Rudder Angle')
        axs[1, 0].legend(loc='best')
    else:
        axs[1, 0].axis('off')

    # --- Plot 5: Cross-track Error Estimation ---
    axs[1, 1].plot(df_own_est['timehat'], df_own_est['ehat'], color='brown', label=r'$\hat{e}$ (Cross-track Est)')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Estimated Error (m)')
    axs[1, 1].set_title('Cross-track Error Estimation')
    axs[1, 1].legend(loc='best')

    # --- Plot 6: Sideslip Estimation ---
    axs[1, 2].plot(df_own_est['timehat'], np.degrees(df_own_est['betahat']), color='olive', label=r'$\hat{\beta}$ (Sideslip Est)')
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].set_ylabel('Sideslip Angle (degrees)')
    axs[1, 2].set_title('Sideslip Estimation')
    axs[1, 2].legend(loc='best')

    # Voorkom overlap door specifiek ruimte voor de suptitle over te laten
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    
    save_path = os.path.join(output_dir, 'combined_6_metrics.png')
    plt.savefig(save_path, dpi=300)
    print(f"Plot saved to {save_path}")
    
    plt.show()

if __name__ == '__main__':
    main()