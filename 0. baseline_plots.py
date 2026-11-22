import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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
    """Sorteert de dataset op tijd, verwijdert t=0 en NaN/Inf, en reset de index."""
    df = df.sort_values(by=time_col)
    df = df[df[time_col] > 0]
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=[time_col])
    return df.reset_index(drop=True)

def compute_vessel_distance(df_own, df_tgt):
    """Berekent de center-to-center afstand uitgelijnd op exacte tijdstippen."""
    merged = pd.merge_asof(df_own, df_tgt, on='t', suffixes=('_own', '_tgt'), direction='nearest')
    dist = np.sqrt(
        (merged['x_true_own'] - merged['x_true_tgt'])**2 +
        (merged['y_true_own'] - merged['y_true_tgt'])**2
    )
    return merged['t'], dist

def plot_run_metrics(run_id, title_suffix="", show_plot=False):
    output_dir = os.path.join('output', f'run_{run_id}')
    try:
        df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')), 't')
        df_own_est = clean_data(pd.read_csv(os.path.join(output_dir, 'Own_estimation.csv')), 'timehat')
        df_other2 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other2.csv')), 't')
        df_other4 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other4.csv')), 't')
    except FileNotFoundError as e:
        print(f"Skipping run_{run_id}: {e}")
        return

    fig = plt.figure(figsize=(18, 10))
    suptitle = f'Sensitivity Analysis: Multi-Metric Overview (Run {run_id}'
    if title_suffix:
        suptitle += f' — {title_suffix}'
    suptitle += ')'
    fig.suptitle(suptitle, fontsize=16, fontweight='bold')

    gs = fig.add_gridspec(2, 6)
    ax1 = fig.add_subplot(gs[0, 0:2])  # Heading Tracking
    ax2 = fig.add_subplot(gs[0, 2:4])  # Safety Margin
    ax3 = fig.add_subplot(gs[0, 4:6])  # Rudder Angle
    ax4 = fig.add_subplot(gs[1, 1:3])  # Cross-track
    ax5 = fig.add_subplot(gs[1, 3:5])  # Sideslip

    # --- Plot 1: Heading Tracking (True vs Measured vs Reference) ---
    if 'psi_noisy' in df_own.columns:
        diff = np.max(np.abs(df_own['psi_noisy'] - df_own['psi']))
        if diff > 1e-4:
            ax1.plot(
                df_own['t'], np.degrees(df_own['psi_noisy']),
                label=r'$\psi_{\mathrm{meas}}$ (Measured)',
                color='lightsteelblue', alpha=0.7, linewidth=0.9
            )
    ax1.plot(df_own['t'], np.degrees(df_own['psi']), label=r'$\psi$ (Actual)', color='blue')
    ax1.plot(df_own['t'], np.degrees(df_own['psi_d']), label=r'$\psi_d$ (Reference)', color='red', linestyle='--')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Heading (degrees)')
    ax1.set_title('Heading Tracking')
    ax1.legend(loc='best')

    # --- Plot 2: Safety Margin ---
    rho_s = 10.67 / 2
    t2, dist_2 = compute_vessel_distance(df_own, df_other2)
    t4, dist_4 = compute_vessel_distance(df_own, df_other4)

    print(f"Run {run_id:2d} | CPA T2: {dist_2.min():.2f} m (margin {dist_2.min() - rho_s:.2f} m) | "
          f"CPA T4: {dist_4.min():.2f} m (margin {dist_4.min() - rho_s:.2f} m)")

    ax2.plot(t2, dist_2 - rho_s, color='green', label='Own vs Target 2')
    ax2.plot(t4, dist_4 - rho_s, color='purple', label='Own vs Target 4')
    ax2.axhline(0, color='red', linestyle='--', label='Collision Boundary')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel(r'$d - \rho_s$ (m)')
    ax2.set_title('Safety Margin (Vessel to Vessel)')
    ax2.legend(loc='best')

    # --- Plot 3: Rudder Angle ---
    if 'delta' in df_own.columns:
        ax3.plot(df_own['t'], np.degrees(df_own['delta']), color='black', label=r'Rudder Angle $\delta$')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (degrees)')
        ax3.set_title('Control Input: Rudder Angle')
        ax3.legend(loc='best')
    else:
        ax3.axis('off')

    # --- Plot 4: Cross-track Error Estimation ---
    ax4.plot(df_own_est['timehat'], df_own_est['ehat'], color='brown', label=r'$\hat{e}$ (Cross-track Est)')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Estimated Error (m)')
    ax4.set_title('Cross-track Error Estimation')
    ax4.legend(loc='best')

    # --- Plot 5: Sideslip Estimation ---
    ax5.plot(df_own_est['timehat'], np.degrees(df_own_est['betahat']), color='olive', label=r'$\hat{\beta}$ (Sideslip Est)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Sideslip Angle (degrees)')
    ax5.set_title('Sideslip Estimation')
    ax5.legend(loc='best')

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    save_path = os.path.join(output_dir, 'combined_5_metrics.png')
    plt.savefig(save_path, dpi=300)
    print(f"Plot saved to {save_path}")

    if show_plot:
        plt.show()
    else:
        plt.close(fig)

def main():
    # Alle runs uit het verslag (voeg desgewenst meer run-IDs toe)
    runs_to_plot = [
        (0, "Baseline"),
        (3, r"GPS Noise $\sigma = 10.0$ m"),
        (6, r"Heading Noise $\sigma = 3.0^\circ$"),
        (8, r"Wind $10.0$ m/s at $90^\circ$"),
        (11, r"Current $1.0$ m/s at $90^\circ$"),
        (15, r"Waves $H_s = 2.0$ m at $90^\circ$"),
    ]
    for run_id, label in runs_to_plot:
        plot_run_metrics(run_id, title_suffix=label, show_plot=False)

if __name__ == '__main__':
    main()