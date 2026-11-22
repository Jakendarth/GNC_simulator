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

def main():
    runs = ['run_1', 'run_2', 'run_3']
    noise_labels = ['0.1 m', '5.0 m', '10.0 m']
    colors = ['blue', 'orange', 'red']

    fig, axs = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Navigation Performance & GPS Error Correlation (0.1 m, 5.0 m, 10.0 m)', fontsize=16, fontweight='bold')

    rho_s = 10.67 / 2  # Veiligheidsmarge (B/2)

    for i, run_dir in enumerate(runs):
        output_dir = os.path.join('output', run_dir)
        try:
            df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')), 't')
            df_other2 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other2.csv')), 't')
            df_other4 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other4.csv')), 't')
        except FileNotFoundError:
            print(f"Skipping {run_dir}: Data niet compleet.")
            continue

        # --- Plot 1: Distance to Vessels (Safety Margin) ---
        merged_2 = pd.merge_asof(df_own, df_other2, on='t', suffixes=('_own', '_tgt'), direction='nearest')
        dist_2 = np.sqrt(
            (merged_2['x_true_own'] - merged_2['x_true_tgt'])**2 +
            (merged_2['y_true_own'] - merged_2['y_true_tgt'])**2
        )
        axs[0, 0].plot(merged_2['t'], dist_2 - rho_s, label=f'T2 ({noise_labels[i]})', color=colors[i], linestyle='-')

        merged_4 = pd.merge_asof(df_own, df_other4, on='t', suffixes=('_own', '_tgt'), direction='nearest')
        dist_4 = np.sqrt(
            (merged_4['x_true_own'] - merged_4['x_true_tgt'])**2 +
            (merged_4['y_true_own'] - merged_4['y_true_tgt'])**2
        )
        axs[0, 0].plot(merged_4['t'], dist_4 - rho_s, label=f'T4 ({noise_labels[i]})', color=colors[i], linestyle='--')

        # --- Plot 2: Heading Tracking ---
        axs[0, 1].plot(df_own['t'], np.degrees(df_own['psi']), label=f'Actual ({noise_labels[i]})', color=colors[i])
        if i == 0:
            axs[0, 1].plot(df_own['t'], np.degrees(df_own['psi_d']), label='Reference', color='black', linestyle='--')

        # --- Plot 3: Path Planning Accuracy (Y-Position) ---
        axs[1, 0].plot(df_own['t'], df_own['y_true'], label=f'Actual Y ({noise_labels[i]})', color=colors[i])
        if i == 0:
            if 'y_d' in df_own.columns:
                axs[1, 0].plot(df_own['t'], df_own['y_d'], color='black', linestyle='--', label='Planned Reference Path (y_d)')
            else:
                axs[1, 0].axhline(df_own['y_true'].iloc[0], color='black', linestyle='--', label='Reference Path')

        # --- Plot 4: Instantaneous GPS Error vs Safety Margin (Target 2) ---
        gps_error_magnitude = np.sqrt(
            (merged_2['x_noisy_own'] - merged_2['x_true_own'])**2 +
            (merged_2['y_noisy_own'] - merged_2['y_true_own'])**2
        )
        axs[1, 1].scatter(gps_error_magnitude, dist_2 - rho_s, label=f'GPS {noise_labels[i]}', color=colors[i], alpha=0.3, s=10)

    axs[0, 0].set_title('Safety Margin (Distance to Targets 2 & 4)')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel(r'$d - \rho_s$ (m)')
    axs[0, 0].axhline(0, color='red', linestyle=':', label='Collision Boundary')
    axs[0, 0].legend(ncol=2)

    axs[0, 1].set_title('Heading Tracking')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Heading (degrees)')
    axs[0, 1].legend()

    axs[1, 0].set_title('Path Planning Accuracy (Y-Position)')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Y Position (m)')
    axs[1, 0].legend()

    axs[1, 1].set_title('Correlation: GPS Error vs Safety Margin (T2)')
    axs[1, 1].set_xlabel('Instantaneous GPS Error Magnitude (m)')
    axs[1, 1].set_ylabel(r'Safety Margin $d - \rho_s$ (m)')
    axs[1, 1].axhline(0, color='red', linestyle=':', label='Collision Boundary')
    leg = axs[1, 1].legend()
    for lh in leg.legend_handles:
        lh.set_alpha(1)

    plt.tight_layout()
    os.makedirs('output', exist_ok=True)
    plt.savefig(os.path.join('output', 'Navigation_Comparison_with_Error.png'), dpi=300)
    plt.show()

if __name__ == '__main__':
    main()