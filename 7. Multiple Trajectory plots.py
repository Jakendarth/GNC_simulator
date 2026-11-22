import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 11,
    'axes.labelsize': 12,
    'legend.fontsize': 9,
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
                        U[i, j] = -u_profiel
                        V[i, j] = 0.0
    return U, V

def plot_sensitivity_group(run_ids, dist_type='none', dist_vals=[], dist_angle=0, title_prefix="Trajectory"):
    target_names = ['Other2', 'Other4']
    colors = ['blue', 'orange', 'red']

    fig, ax = plt.subplots(figsize=(12, 8))
    draw_environment(ax)

    # Omgevingsvectoren (Wind of Stroming)
    if dist_type.lower() in ['wind', 'current'] and len(dist_vals) > 0:
        max_val_str = dist_vals[-1].split(' ')[0].replace('m/s', '')
        try:
            max_val = float(max_val_str)
            x_grid = np.linspace(200, 5800, 25)
            y_grid = np.linspace(-2350, -1650, 7)
            X, Y = np.meshgrid(x_grid, y_grid)
            U, V = calculate_vector_field(X, Y, dist_type, max_val, dist_angle)

            c = 'cyan' if dist_type.lower() == 'current' else 'gray'
            label_str = f'{dist_type.capitalize()} (up to {dist_vals[-1]}, {dist_angle}°)'
            ax.quiver(X, Y, U, V, color=c, alpha=0.6, scale=50, width=0.003, label=label_str)
        except ValueError:
            pass

    for i, run_id in enumerate(run_ids):
        output_dir = os.path.join('output', f'run_{run_id}')
        try:
            df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')))
            targets_data = {}
            for tgt in target_names:
                tgt_path = os.path.join(output_dir, f'{tgt}.csv')
                if os.path.exists(tgt_path):
                    targets_data[tgt] = clean_data(pd.read_csv(tgt_path))
        except Exception as e:
            print(f"Data voor Run {run_id} mist of is incompleet: {e}")
            continue

        run_color = colors[i]
        val_label = f"{dist_vals[i]}" if i < len(dist_vals) else f"Run {run_id}"

        ax.plot(df_own['x_true'], df_own['y_true'], color=run_color, linestyle='-', linewidth=1.2, zorder=3, label=f'Own ({val_label})')
        ax.scatter(df_own['x_true'].iloc[0], df_own['y_true'].iloc[0], color=run_color, marker='X', s=50, zorder=6)

        for tgt_name, df_tgt in targets_data.items():
            l_style = '--' if tgt_name == 'Other2' else ':'
            tgt_label = f"T2 ({val_label})" if tgt_name == 'Other2' else f"T4 ({val_label})"
            ax.plot(df_tgt['x_true'], df_tgt['y_true'], color=run_color, linestyle=l_style, linewidth=1.2, zorder=3, label=tgt_label)
            ax.scatter(df_tgt['x_true'].iloc[0], df_tgt['y_true'].iloc[0], color=run_color, marker='X', s=50, zorder=6)

            merged = pd.merge_asof(df_own, df_tgt, on='t', suffixes=('_own', '_tgt'), direction='nearest')
            merged['dist'] = np.sqrt(
                (merged['x_true_own'] - merged['x_true_tgt'])**2 +
                (merged['y_true_own'] - merged['y_true_tgt'])**2
            )

            t_cpa = merged.loc[merged['dist'].idxmin(), 't']
            idx_o = (df_own['t'] - t_cpa).abs().idxmin()
            idx_t = (df_tgt['t'] - t_cpa).abs().idxmin()

            x_o, y_o = df_own.loc[idx_o, 'x_true'], df_own.loc[idx_o, 'y_true']
            x_t, y_t = df_tgt.loc[idx_t, 'x_true'], df_tgt.loc[idx_t, 'y_true']

            ax.scatter(x_o, y_o, color=run_color, marker='o', s=40, zorder=4)
            ax.scatter(x_t, y_t, color=run_color, marker='o', s=40, zorder=4)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_xlim(0, 6000)
    ax.set_ylim(-4000, 0)
    ax.set_yticks(np.arange(-4000, 500, 500))
    ax.set_title(title_prefix, fontweight='bold', pad=10)
    ax.legend(loc="lower left", framealpha=1, edgecolor='black', ncol=3, fontsize=8)

    plt.tight_layout()
    save_dir = os.path.join('output', 'analysis_plots')
    os.makedirs(save_dir, exist_ok=True)
    filename = os.path.join(save_dir, f'{dist_type}_grouped_trajectory.png')
    plt.savefig(filename, dpi=300)
    print(f"Opgeslagen: {filename}")
    plt.close(fig)

if __name__ == '__main__':
    # Group 1: GPS Noise (1, 2, 3)
    plot_sensitivity_group([1, 2, 3], dist_type='GPS', dist_vals=['0.1 m', '5.0 m', '10.0 m'], title_prefix="Trajectory Comparison (GPS Noise)")

    # Group 2: Heading Noise (4, 5, 6)
    plot_sensitivity_group([4, 5, 6], dist_type='Heading', dist_vals=['0.1°', '1.0°', '3.0°'], title_prefix="Trajectory Comparison (Heading Noise)")

    # Group 3: Wind (7, 8, 9)
    plot_sensitivity_group([7, 8, 9], dist_type='wind', dist_vals=['5.0 m/s', '10.0 m/s', '15.0 m/s'], dist_angle=90, title_prefix="Trajectory Comparison (Wind at 90°)")

    # Group 4: Current (10, 11, 12)
    plot_sensitivity_group([10, 11, 12], dist_type='current', dist_vals=['0.5 m/s', '1.0 m/s', '1.5 m/s'], dist_angle=90, title_prefix="Trajectory Comparison (Current at 90°)")

    # Group 5: Waves (13, 14, 15)
    plot_sensitivity_group([13, 14, 15], dist_type='wave', dist_vals=['0.5 m', '1.0 m', '2.0 m'], dist_angle=90, title_prefix="Trajectory Comparison (Waves at 90°)")