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
    runs = ['run_4', 'run_5', 'run_6']
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
            # Extra inladen van Target 4
            df_other4 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other4.csv')), 't')
        except FileNotFoundError:
            print(f"Skipping {run_dir}: Own.csv, Other2.csv of Other4.csv niet gevonden.")
            continue
            
        # --- Plot 1: Distance to Vessels (Safety Margin) ---
        # Berekening voor Target 2
        min_len_2 = min(len(df_own), len(df_other2))
        dist_2 = np.sqrt((df_own['x_true'].iloc[:min_len_2] - df_other2['x_true'].iloc[:min_len_2])**2 + 
                         (df_own['y_true'].iloc[:min_len_2] - df_other2['y_true'].iloc[:min_len_2])**2)
        axs[0].plot(df_own['t'].iloc[:min_len_2], dist_2 - rho_s, label=f'T2 {noise_labels[i]}', color=colors[i], linestyle='-')
        
        # Berekening voor Target 4
        min_len_4 = min(len(df_own), len(df_other4))
        dist_4 = np.sqrt((df_own['x_true'].iloc[:min_len_4] - df_other4['x_true'].iloc[:min_len_4])**2 + 
                         (df_own['y_true'].iloc[:min_len_4] - df_other4['y_true'].iloc[:min_len_4])**2)
        axs[0].plot(df_own['t'].iloc[:min_len_4], dist_4 - rho_s, label=f'T4 {noise_labels[i]}', color=colors[i], linestyle='--')
        
        # --- Plot 2: Heading Tracking ---
        axs[1].plot(df_own['t'], np.degrees(df_own['psi']), label=f'Actual ({noise_labels[i]})', color=colors[i])
        if i == 0:
            axs[1].plot(df_own['t'], np.degrees(df_own['psi_d']), label='Reference', color='black', linestyle='--')
            
        # --- Plot 3: Path Planning Accuracy (Tracking Deviation in Y) ---
        axs[2].plot(df_own['t'], df_own['y_true'], label=f'Actual Y ({noise_labels[i]})', color=colors[i])
        
        # NIEUW: Gebruik hier de daadwerkelijk voorspelde padlijn y_d uit de CSV
        # We plotten deze referentie alleen voor run 1 (i==0) zodat de grafiek niet te vol wordt.
        if i == 0:
            if 'y_d' in df_own.columns:
                axs[2].plot(df_own['t'], df_own['y_d'], color='black', linestyle='--', label='Planned Reference Path (y_d)')
            else:
                # Als y_d toch niet in de CSV staat, geef een fallback of melding
                print("Let op: kolom 'y_d' niet gevonden in CSV. Fallback naar statische lijn.")
                axs[2].axhline(df_own['y_true'].iloc[0], color='black', linestyle='--', label='Reference Path (Static Fallback)')

    axs[0].set_title('Safety Margin (Distance to Targets 2 & 4)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel(r'$d - \rho_s$ (m)')
    axs[0].axhline(0, color='red', linestyle=':', label='Collision Boundary')
    # Gebruik 2 kolommen voor de legenda, anders wordt deze te lang met 6 lijnen
    axs[0].legend(ncol=2)
    
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