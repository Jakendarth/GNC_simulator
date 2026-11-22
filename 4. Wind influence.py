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
    df = df.sort_values(by=time_col)
    df = df[df[time_col] > 0]
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=[time_col])
    return df.reset_index(drop=True)

def compute_vessel_distance(df_own, df_tgt):
    merged = pd.merge_asof(df_own, df_tgt, on='t', suffixes=('_own', '_tgt'), direction='nearest')
    dist = np.sqrt(
        (merged['x_true_own'] - merged['x_true_tgt'])**2 +
        (merged['y_true_own'] - merged['y_true_tgt'])**2
    )
    return merged['t'], dist

def main():
    runs = ['run_7', 'run_8', 'run_9']
    noise_labels = ['5.0 m/s', '10.0 m/s', '15.0 m/s']
    colors = ['blue', 'orange', 'red']

    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    fig.suptitle('Navigation Performance Comparison (Wind Speeds at 90 deg)', fontsize=16, fontweight='bold')

    rho_s = 10.67 / 2  # Veiligheidsmarge

    for i, run_dir in enumerate(runs):
        output_dir = os.path.join('output', run_dir)
        try:
            df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')), 't')
            df_other2 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other2.csv')), 't')
            df_other4 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other4.csv')), 't')
        except FileNotFoundError:
            print(f"Skipping {run_dir}: CSV niet gevonden.")
            continue

        # --- Plot 1: Distance to Vessels ---
        t2, dist_2 = compute_vessel_distance(df_own, df_other2)
        t4, dist_4 = compute_vessel_distance(df_own, df_other4)
        print(f"[Wind {noise_labels[i]:8s}] CPA T2: {dist_2.min():.2f} m | CPA T4: {dist_4.min():.2f} m")

        axs[0].plot(t2, dist_2 - rho_s, label=f'T2 ({noise_labels[i]})', color=colors[i], linestyle='-')
        axs[0].plot(t4, dist_4 - rho_s, label=f'T4 ({noise_labels[i]})', color=colors[i], linestyle='--')

        # --- Plot 2: Heading Tracking ---
        axs[1].plot(df_own['t'], np.degrees(df_own['psi']), label=f'Actual ({noise_labels[i]})', color=colors[i])
        if i == 0:
            axs[1].plot(df_own['t'], np.degrees(df_own['psi_d']), label='Reference', color='black', linestyle='--')

        # --- Plot 3: Path Planning Accuracy ---
        axs[2].plot(df_own['t'], df_own['y_true'], label=f'Actual Y ({noise_labels[i]})', color=colors[i])
        if i == 0:
            if 'y_d' in df_own.columns:
                axs[2].plot(df_own['t'], df_own['y_d'], color='black', linestyle='--', label='Planned Reference Path (y_d)')
            else:
                axs[2].axhline(df_own['y_true'].iloc[0], color='black', linestyle='--', label='Reference Path (Static Fallback)')

    axs[0].set_title('Safety Margin (Distance to Targets 2 & 4)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel(r'$d - \rho_s$ (m)')
    axs[0].axhline(0, color='red', linestyle=':', label='Collision Boundary')
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
    plt.savefig(os.path.join('output', 'Navigation_Comparison_Wind.png'), dpi=300)
    plt.show()

if __name__ == '__main__':
    main()