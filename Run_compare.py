import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def clean_data(df, time_col='t'):
    """Sorteert de data chronologisch en verwijdert de t=0 initialisatie."""
    df = df.sort_values(by=time_col).reset_index(drop=True)
    return df[df[time_col] > 0]

def compare_runs(base_output_dir='output', num_runs=5, target_vessel='Own'):
    plt.style.use('seaborn-v0_8-darkgrid')
    
    # ---------------------------------------------------------
    # 1. GECOMBINEERDE PLOT (Alles over elkaar heen)
    # ---------------------------------------------------------
    plt.figure(figsize=(12, 8))
    colors = plt.cm.viridis(np.linspace(0, 0.9, num_runs))
    
    for run_id in range(1, num_runs + 1):
        file_path = os.path.join(base_output_dir, f'run_{run_id}', f'{target_vessel}.csv')
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            df = clean_data(df, 't')
            
            # Plot de x_true en y_true coördinaten
            plt.plot(df['x_true'], df['y_true'], label=f'Run {run_id}', color=colors[run_id-1], alpha=0.7, linewidth=2)
            
            # Markeer het start- en eindpunt
            plt.scatter(df['x_true'].iloc[0], df['y_true'].iloc[0], color=colors[run_id-1], marker='o', s=50, edgecolors='black', zorder=5)
            plt.scatter(df['x_true'].iloc[-1], df['y_true'].iloc[-1], color=colors[run_id-1], marker='s', s=50, edgecolors='black', zorder=5)

    plt.title(f'Gecombineerde Trajecten over {num_runs} Runs ({target_vessel} Vessel)', fontsize=16, fontweight='bold')
    plt.xlabel('X Positie (m)', fontsize=14)
    plt.ylabel('Y Positie (m)', fontsize=14)
    plt.legend(loc='best', frameon=True, shadow=True)
    plt.grid(True, alpha=0.3)
    plt.axis('equal') 
    
    plt.tight_layout()
    plt.savefig(os.path.join(base_output_dir, f'combined_trajectories_{target_vessel}.png'), dpi=300)
    plt.show()

    # ---------------------------------------------------------
    # 2. LOSSE PLOTS (Subplots per run)
    # ---------------------------------------------------------
    fig, axes = plt.subplots(num_runs, 1, figsize=(10, 3 * num_runs), sharex=True, sharey=True)
    if num_runs == 1: 
        axes = [axes] 
        
    for run_id in range(1, num_runs + 1):
        ax = axes[run_id - 1]
        file_path = os.path.join(base_output_dir, f'run_{run_id}', f'{target_vessel}.csv')
        
        if os.path.exists(file_path):
            df = pd.read_csv(file_path)
            df = clean_data(df, 't')
            
            ax.plot(df['x_true'], df['y_true'], color=colors[run_id-1], linewidth=2)
            ax.scatter(df['x_true'].iloc[0], df['y_true'].iloc[0], color=colors[run_id-1], marker='o', s=50, edgecolors='black', zorder=5)
            ax.scatter(df['x_true'].iloc[-1], df['y_true'].iloc[-1], color=colors[run_id-1], marker='s', s=50, edgecolors='black', zorder=5)
            
            ax.set_title(f'Traject Run {run_id}', fontsize=12)
            ax.set_ylabel('Y (m)')
            ax.grid(True, alpha=0.3)
            ax.axis('equal')
            
    axes[-1].set_xlabel('X Positie (m)', fontsize=14)
    plt.tight_layout()
    plt.savefig(os.path.join(base_output_dir, f'individual_trajectories_{target_vessel}.png'), dpi=300)
    plt.show()

if __name__ == '__main__':
    compare_runs(base_output_dir='output', num_runs=5, target_vessel='Own')