import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Strakke, academische opmaak gebaseerd op de referentie
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
    """Sorteer chronologisch en verwijder de t=0 initialisatie uitschieters."""
    df = df.sort_values(by=time_col).reset_index(drop=True)
    return df[df[time_col] > 0]

def draw_environment(ax):
    """Tekent de landmassa's, 2000m naar beneden verschoven zodat het kanaal centreert rond Y = -2000."""
    land_color = '#F5F5DC'  # Beige/zand kleur
    
    # Kanaal is 750m breed (-2375 tot -1625)
    # 1. Bodem-links (Zuidelijke oever, deel 1) - start op -4000, hoogte 1625 -> rand op -2375
    ax.add_patch(patches.Rectangle((3000, -4000), 1400, 1625, facecolor=land_color, edgecolor='black', zorder=1))
    
    # 2. Top-links (Noordelijke oever, deel 1) - start op -1625, hoogte 1625 -> rand op 0
    ax.add_patch(patches.Rectangle((3000, -1625), 1400, 1625, facecolor=land_color, edgecolor='black', zorder=1))
    
    # 3. Bodem-rechts (Zuidelijke oever, deel 2)
    ax.add_patch(patches.Rectangle((5200, -4000), 800, 1625, facecolor=land_color, edgecolor='black', zorder=1))
    
    # 4. Top-rechts (Noordelijke oever, deel 2)
    ax.add_patch(patches.Rectangle((5200, -1625), 800, 1625, facecolor=land_color, edgecolor='black', zorder=1))

def plot_sensitivity_run(run_id, dist_type='none', dist_val=0, dist_angle=0):
    output_dir = f'output/run_{run_id}'
    target_names = ['Other2', 'Other4']
    
    # 1. Data Inladen
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
    
    # 2. Teken de exacte kanaal-omgeving
    draw_environment(ax)

    # 3. Omgevingsvectoren (Pijlenveld voor Wind of Stroming)
    if dist_type in ['wind', 'current'] and dist_val > 0:
        # Teken stroompijlen alleen in het watergebied
        X, Y = np.meshgrid(np.linspace(100, 5900, 20), np.linspace(-2300, -1700, 5))
        U = dist_val * np.cos(np.radians(dist_angle))
        V = dist_val * np.sin(np.radians(dist_angle))
        
        c = 'cyan' if dist_type == 'current' else 'gray'
        label_str = f'{dist_type.capitalize()} ({dist_val} m/s, {dist_angle}°)'
        ax.quiver(X, Y, U, V, color=c, alpha=0.6, scale=50, width=0.0025, label=label_str)

    # 4. Trajectoriën (Ground Truth) Tekenen
    colors = {'Own': 'steelblue', 'Other2': 'forestgreen', 'Other4': 'mediumvioletred'}
    labels = {'Own': 'Own Vessel', 'Other2': 'Other Vessel 1', 'Other4': 'Other Vessel 2'}
    
    # Eigen schip plotten
    ax.plot(df_own['x_true'], df_own['y_true'], color=colors['Own'], linestyle='--', linewidth=1.2, zorder=3, label=labels['Own'])
    
    for tgt_name, df_tgt in targets_data.items():
        c = colors.get(tgt_name, 'red')
        ax.plot(df_tgt['x_true'], df_tgt['y_true'], color=c, linestyle='--', linewidth=1.2, zorder=3, label=labels.get(tgt_name, tgt_name))
        
        # --- CPA & TIJDSTIPPEN BEREKENEN ---
        merged = pd.merge_asof(df_own, df_tgt, on='t', suffixes=('_own', '_tgt'))
        merged['dist'] = np.sqrt((merged['x_true_own'] - merged['x_true_tgt'])**2 + 
                                 (merged['y_true_own'] - merged['y_true_tgt'])**2)
        
        # Tijdstippen: Kruising (CPA), 1 minuut ervoor, 1 minuut erna
        t_cpa = merged.loc[merged['dist'].idxmin(), 't']
        t_points = [t_cpa - 150, t_cpa, t_cpa + 150] 
        
        for t_val in t_points:
            if t_val < df_own['t'].min() or t_val > df_own['t'].max(): continue
            
            # Vind posities op dit tijdstip
            idx_o = (df_own['t'] - t_val).abs().idxmin()
            idx_t = (df_tgt['t'] - t_val).abs().idxmin()
            
            x_o, y_o = df_own['x_true'].iloc[idx_o], df_own['y_true'].iloc[idx_o]
            x_t, y_t = df_tgt['x_true'].iloc[idx_t], df_tgt['y_true'].iloc[idx_t]
            
            # Teken de scheepsposities als dikke stippen/ellipsen
            ax.scatter(x_o, y_o, color=colors['Own'], marker='o', s=80, zorder=4)
            ax.scatter(x_t, y_t, color=c, marker='o', s=80, zorder=4)
            
            # Tekstbox met tijdstip
            bbox_props_own = dict(boxstyle="round,pad=0.3", fc="white", ec=colors['Own'], lw=1.5)
            bbox_props_tgt = dict(boxstyle="round,pad=0.3", fc="white", ec=c, lw=1.5)
            
            ax.annotate(f"t={int(t_val)} s", (x_o, y_o), xytext=(0, -25), textcoords='offset points', 
                        ha='center', fontsize=9, bbox=bbox_props_own, zorder=5)
            ax.annotate(f"t={int(t_val)} s", (x_t, y_t), xytext=(0, 15), textcoords='offset points', 
                        ha='center', fontsize=9, bbox=bbox_props_tgt, zorder=5)

    # 5. Opmaak en Weergave (Assen verschoven naar -4000 tot 0)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_xlim(0, 6000)
    ax.set_ylim(-4000, 0)
    ax.set_yticks(np.arange(-4000, 500, 500))
    
    title = f"Run {run_id}: Trajectory"
    if dist_type != 'none': title += f" | {dist_type.capitalize()} = {dist_val}"
    ax.set_title(title, fontweight='bold', pad=10)
    
    # Legenda linksonder
    ax.legend(loc="lower left", framealpha=1, edgecolor='black')
    
    plt.tight_layout()
    os.makedirs('output/analysis_plots', exist_ok=True)
    plt.savefig(f'output/analysis_plots/Run_{run_id}_{dist_type}_trajectory.png', dpi=300)
    print(f"Opgeslagen: output/analysis_plots/Run_{run_id}_{dist_type}_trajectory.png")
    plt.show()

if __name__ == '__main__':
    # Test hier je runs!
    
    # Run 0 (Baseline)
    plot_sensitivity_run(run_id=6, dist_type='wind', dist_val=5.0, dist_angle=90)

    plot_sensitivity_run(run_id=13, dist_type='current', dist_val=0.5, dist_angle=90)
    # Run 8: Voorbeeld voor winddwarsligging:
    # plot_sensitivity_run(run_id=8, dist_type='wind', dist_val=10.0, dist_angle=90)