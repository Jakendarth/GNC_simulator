import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'legend.fontsize': 10,
    'axes.grid': True,
    'grid.linestyle': '--',
    'grid.alpha': 0.5
})

def clean_data(df, time_col='t'):
    """Sorteer chronologisch, verwijder t=0 en NaN/Inf, en reset de index."""
    df = df.sort_values(by=time_col)
    df = df[df[time_col] > 0]
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=[time_col, 'x_true', 'y_true'])
    return df.reset_index(drop=True)

def draw_environment(ax):
    """Tekent de landmassa's met een kanaal van 750 m breed rond Y = -2000 m."""
    land_color = '#F5F5DC'
    ax.add_patch(patches.Rectangle((3000, -4000), 1400, 1625, facecolor=land_color, edgecolor='black', zorder=1))
    ax.add_patch(patches.Rectangle((3000, -1625), 1400, 1625, facecolor=land_color, edgecolor='black', zorder=1))
    ax.add_patch(patches.Rectangle((5200, -4000), 800, 1625, facecolor=land_color, edgecolor='black', zorder=1))
    ax.add_patch(patches.Rectangle((5200, -1625), 800, 1625, facecolor=land_color, edgecolor='black', zorder=1))

def calculate_vector_field(X, Y, dist_type, dist_val, dist_angle):
    """Berekent het (U, V) pijlenveld voor zee (x < 3000) en binnenwater (x >= 3000)."""
    U = np.zeros_like(X, dtype=float)
    V = np.zeros_like(Y, dtype=float)
    dt_lower = dist_type.lower()

    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            x_val = X[i, j]
            y_val = Y[i, j]

            if dt_lower == 'wind':
                factor = 1.0 if x_val < 3000 else 0.4
                U[i, j] = (dist_val * factor) * np.cos(np.radians(dist_angle))
                V[i, j] = (dist_val * factor) * np.sin(np.radians(dist_angle))

            elif dt_lower == 'current':
                if x_val < 3000:
                    U[i, j] = dist_val * np.cos(np.radians(dist_angle))
                    V[i, j] = dist_val * np.sin(np.radians(dist_angle))
                else:
                    w = 750.0
                    y_local = y_val - (-2000.0)
                    if abs(y_local) < (w / 2):
                        u_profiel = dist_val * (1.0 - (2.0 * y_local / w)**2)
                        U[i, j] = -u_profiel  # Rivier stroomt richting zee (-x)
                        V[i, j] = 0.0
    return U, V

def plot_sensitivity_run(run_id, dist_type='None', dist_val=0, dist_angle=0, unit=''):
    output_dir = os.path.join('output', f'run_{run_id}')
    target_names = ['Other2', 'Other4']

    try:
        df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')))
        targets_data = {}
        for tgt in target_names:
            tgt_path = os.path.join(output_dir, f'{tgt}.csv')
            if os.path.exists(tgt_path):
                targets_data[tgt] = clean_data(pd.read_csv(tgt_path))
    except Exception as e:
        print(f"Data voor Run {run_id} mist: {e}")
        return

    fig, ax = plt.subplots(figsize=(12, 8))
    draw_environment(ax)

    # Dynamische omgevingsvectoren (Wind of Stroming)
    if dist_type.lower() in ['wind', 'current'] and dist_val > 0:
        x_grid = np.linspace(200, 5800, 25)
        y_grid = np.linspace(-2350, -1650, 7)
        X, Y = np.meshgrid(x_grid, y_grid)
        U, V = calculate_vector_field(X, Y, dist_type, dist_val, dist_angle)
        c = 'cyan' if dist_type.lower() == 'current' else 'gray'
        label_str = f'{dist_type.capitalize()} ({dist_val} m/s, {dist_angle}°)'
        ax.quiver(X, Y, U, V, color=c, alpha=0.6, scale=50, width=0.003, label=label_str)

    colors = {'Own': 'steelblue', 'Other2': 'forestgreen', 'Other4': 'mediumvioletred'}
    labels = {'Own': 'Own Vessel', 'Other2': 'Other Vessel 1', 'Other4': 'Other Vessel 2'}

    ax.plot(df_own['x_true'], df_own['y_true'], color=colors['Own'], linestyle='--', linewidth=1.2, zorder=3, label=labels['Own'])
    ax.scatter(df_own['x_true'].iloc[0], df_own['y_true'].iloc[0], color=colors['Own'], marker='X', s=100, zorder=6, label=f"Start {labels['Own']}")

    cpa_info = []
    for tgt_name, df_tgt in targets_data.items():
        c = colors.get(tgt_name, 'red')
        ax.plot(df_tgt['x_true'], df_tgt['y_true'], color=c, linestyle='--', linewidth=1.2, zorder=3, label=labels.get(tgt_name, tgt_name))
        ax.scatter(df_tgt['x_true'].iloc[0], df_tgt['y_true'].iloc[0], color=c, marker='X', s=100, zorder=6, label=f"Start {labels.get(tgt_name, tgt_name)}")

        merged = pd.merge_asof(df_own, df_tgt, on='t', suffixes=('_own', '_tgt'), direction='nearest')
        merged['dist'] = np.sqrt(
            (merged['x_true_own'] - merged['x_true_tgt'])**2 +
            (merged['y_true_own'] - merged['y_true_tgt'])**2
        )

        idx_min = merged['dist'].idxmin()
        t_cpa = merged.loc[idx_min, 't']
        d_cpa = merged.loc[idx_min, 'dist']
        cpa_info.append(f"{tgt_name}: {d_cpa:.2f} m (t={int(t_cpa)} s)")

        t_points = [t_cpa - 150, t_cpa, t_cpa + 150]
        for t_val in t_points:
            if t_val < df_own['t'].min() or t_val > df_own['t'].max():
                continue

            idx_o = (df_own['t'] - t_val).abs().idxmin()
            idx_t = (df_tgt['t'] - t_val).abs().idxmin()

            x_o, y_o = df_own.loc[idx_o, 'x_true'], df_own.loc[idx_o, 'y_true']
            x_t, y_t = df_tgt.loc[idx_t, 'x_true'], df_tgt.loc[idx_t, 'y_true']

            ax.scatter(x_o, y_o, color=colors['Own'], marker='o', s=80, zorder=4)
            ax.scatter(x_t, y_t, color=c, marker='o', s=80, zorder=4)

            bbox_props_own = dict(boxstyle="round,pad=0.3", fc="white", ec=colors['Own'], lw=1.5)
            bbox_props_tgt = dict(boxstyle="round,pad=0.3", fc="white", ec=c, lw=1.5)

            ax.annotate(f"t={int(t_val)} s", (x_o, y_o), xytext=(0, -25), textcoords='offset points',
                        ha='center', fontsize=9, bbox=bbox_props_own, zorder=5)
            ax.annotate(f"t={int(t_val)} s", (x_t, y_t), xytext=(0, 15), textcoords='offset points',
                        ha='center', fontsize=9, bbox=bbox_props_tgt, zorder=5)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_xlim(0, 6000)
    ax.set_ylim(-4000, 0)
    ax.set_yticks(np.arange(-4000, 500, 500))

    if dist_type.lower() == 'none':
        title = f"Run {run_id}: Trajectory (Baseline)"
    else:
        val_str = f"{dist_val} {unit}".strip()
        title = f"Run {run_id}: Trajectory | {dist_type} = {val_str}"
    ax.set_title(title, fontweight='bold', pad=10)

    ax.legend(loc="lower left", framealpha=1, edgecolor='black', ncol=2)

    plt.tight_layout()
    save_dir = os.path.join('output', 'analysis_plots')
    os.makedirs(save_dir, exist_ok=True)
    save_path = os.path.join(save_dir, f'Run_{run_id}_{dist_type}_trajectory.png')
    plt.savefig(save_path, dpi=300)
    print(f"Opgeslagen: {save_path} | " + " | ".join(cpa_info))
    plt.close(fig)

if __name__ == '__main__':
    # Run 0: Baseline
    plot_sensitivity_run(run_id=0, dist_type='None')

    # Runs 1, 2, 3: GPS-ruis
    plot_sensitivity_run(run_id=1, dist_type='GPS', dist_val=0.1, unit='m')
    plot_sensitivity_run(run_id=2, dist_type='GPS', dist_val=5.0, unit='m')
    plot_sensitivity_run(run_id=3, dist_type='GPS', dist_val=10.0, unit='m')

    # Runs 4, 5, 6: Heading-ruis
    plot_sensitivity_run(run_id=4, dist_type='Heading', dist_val=0.1, unit='°')
    plot_sensitivity_run(run_id=5, dist_type='Heading', dist_val=1.0, unit='°')
    plot_sensitivity_run(run_id=6, dist_type='Heading', dist_val=3.0, unit='°')

    # Runs 7, 8, 9: Wind (90 deg)
    plot_sensitivity_run(run_id=7, dist_type='wind', dist_val=5.0, dist_angle=90, unit='m/s')
    plot_sensitivity_run(run_id=8, dist_type='wind', dist_val=10.0, dist_angle=90, unit='m/s')
    plot_sensitivity_run(run_id=9, dist_type='wind', dist_val=15.0, dist_angle=90, unit='m/s')

    # Runs 10, 11, 12: Stroming (90 deg)
    plot_sensitivity_run(run_id=10, dist_type='current', dist_val=0.5, dist_angle=90, unit='m/s')
    plot_sensitivity_run(run_id=11, dist_type='current', dist_val=1.0, dist_angle=90, unit='m/s')
    plot_sensitivity_run(run_id=12, dist_type='current', dist_val=1.5, dist_angle=90, unit='m/s')

    # Runs 13, 14, 15: Golfhoogte Hs (90 deg) — gecorrigeerd naar 0.5, 1.0, 2.0 m
    plot_sensitivity_run(run_id=13, dist_type='wave', dist_val=0.5, dist_angle=90, unit='m')
    plot_sensitivity_run(run_id=14, dist_type='wave', dist_val=1.0, dist_angle=90, unit='m')
    plot_sensitivity_run(run_id=15, dist_type='wave', dist_val=2.0, dist_angle=90, unit='m')