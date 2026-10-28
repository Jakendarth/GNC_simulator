#MAKE SURE ALL FILES IN SIMPARAM_WC.json ARE SET TO ZERO (NO DISTURBANCES) BEFORE RUNNING THIS SCRIPT

import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Apply publication-quality settings
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
    """Sorteert de dataset op tijd en verwijdert de initialisatiestap (t=0 of startpositie 0,0)."""
    df = df.sort_values(by=time_col).reset_index(drop=True)
    df = df[df[time_col] > 0]
    return df

def main():
    # Update naar de specifieke run map
    output_dir = 'output/run_3' 
    
    # 1. Laad en schoon de data (Standaard + Estimation)
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
        print(f"Error loading files: {e}. Check of 'python main.py' succesvol is afgerond.")
        return

    # 2. Setup Figure met 6 subplots (2 rijen, 3 kolommen)
    fig, axs = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Baseline 0-Measurement Results (No Disturbances)', fontsize=16, fontweight='bold')

    # --- Plot A: X-Y Trajectories ---
    axs[0, 0].plot(df_own['x_true'], df_own['y_true'], label='Own Vessel', color='blue')
    axs[0, 0].plot(df_other2['x_true'], df_other2['y_true'], label='Target 2', color='green')
    axs[0, 0].plot(df_other4['x_true'], df_other4['y_true'], label='Target 4', color='purple')
    
    # We pakken nu het échte eerste punt na filteren
    axs[0, 0].scatter(df_own['x_true'].iloc[0], df_own['y_true'].iloc[0], marker='o', color='blue', label='Start Own')
    axs[0, 0].set_xlabel('X Position (m)')
    axs[0, 0].set_ylabel('Y Position (m)')
    axs[0, 0].set_title('X-Y Trajectories')
    axs[0, 0].legend()
    axs[0, 0].axis('equal')

    # --- Plot B: Heading (Actual vs Desired) ---
    axs[0, 1].plot(df_own['t'], np.degrees(df_own['psi']), label=r'$\psi$ (Actual)', color='blue')
    axs[0, 1].plot(df_own['t'], np.degrees(df_own['psi_d']), label=r'$\psi_d$ (Reference)', color='red', linestyle='--')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Heading (degrees)')
    axs[0, 1].set_title('Heading Tracking')
    axs[0, 1].legend()

    # --- Plot C: Inter-vessel Distance vs Safe Distance ---
    min_len_2 = min(len(df_own), len(df_other2))
    min_len_4 = min(len(df_own), len(df_other4))
    
    dist_2 = np.sqrt((df_own['x_true'].iloc[:min_len_2] - df_other2['x_true'].iloc[:min_len_2])**2 + 
                     (df_own['y_true'].iloc[:min_len_2] - df_other2['y_true'].iloc[:min_len_2])**2)
    dist_4 = np.sqrt((df_own['x_true'].iloc[:min_len_4] - df_other4['x_true'].iloc[:min_len_4])**2 + 
                     (df_own['y_true'].iloc[:min_len_4] - df_other4['y_true'].iloc[:min_len_4])**2)
    
    rho_s = 10.67 / 2  # Gebaseerd op scheepsbreedte
    
    axs[0, 2].plot(df_own['t'].iloc[:min_len_2], dist_2 - rho_s, color='green', label='Own vs Target 2')
    axs[0, 2].plot(df_own['t'].iloc[:min_len_4], dist_4 - rho_s, color='purple', label='Own vs Target 4')
    axs[0, 2].axhline(0, color='red', linestyle='--', label='Collision Boundary')
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel(r'$d - \rho_s$ (m)')
    axs[0, 2].set_title('Safety Margin (Vessel to Vessel)')
    axs[0, 2].legend()

    # --- Plot D: Bank Distances ---
    axs[1, 0].plot(df_own['t'], df_own['dlp'], label='Port Bank', color='orange')
    axs[1, 0].plot(df_own['t'], df_own['dls'], label='Starboard Bank', color='cyan')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Distance (m)')
    axs[1, 0].set_title('Infrastructure Clearance')
    axs[1, 0].legend()

    # --- Plot E: State Estimation (Heading vs Estimated Heading) ---
    axs[1, 1].plot(df_own['t'], np.degrees(df_own['psi']), label=r'$\psi$ (True)', color='blue', alpha=0.6)
    axs[1, 1].plot(df_own_est['timehat'], np.degrees(df_own_est['psihat']), label=r'$\hat{\psi}$ (Estimated)', color='magenta', linestyle='--')
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('Heading (degrees)')
    axs[1, 1].set_title('Heading Estimation Validation')
    axs[1, 1].legend()

    # --- Plot F: Sideslip & Cross-track Error Estimation ---
    axs[1, 2].plot(df_own_est['timehat'], df_own_est['ehat'], label=r'$\hat{e}$ (Cross-track Est)', color='brown')
    axs[1, 2].plot(df_own_est['timehat'], np.degrees(df_own_est['betahat']), label=r'$\hat{\beta}$ (Sideslip Est deg)', color='olive')
    axs[1, 2].set_xlabel('Time (s)')
    axs[1, 2].set_ylabel('Error / Angle')
    axs[1, 2].set_title('Cross-track & Sideslip Estimation')
    axs[1, 2].legend()

    plt.tight_layout()
    
    # Save and show
    os.makedirs(output_dir, exist_ok=True)
    plt.savefig(os.path.join(output_dir, 'baseline_0_measurement.png'), dpi=300)
    print(f"Plot saved to {output_dir}/baseline_0_measurement.png")
    plt.show()

if __name__ == '__main__':
    main()