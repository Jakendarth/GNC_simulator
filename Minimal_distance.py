import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Strakke, academische opmaak
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
    """Sorteer chronologisch, gooi initialisatie en eventuele kapotte simulatie-data (NaNs) weg."""
    df = df.sort_values(by=time_col).reset_index(drop=True)
    df = df[df[time_col] > 0]
    df = df.replace([np.inf, -np.inf], np.nan).dropna(subset=['x_true', 'y_true'])
    return df

def plot_min_distance_bar_chart(run_ids, labels, category_name):
    # Lijsten om de minimale afstanden in op te slaan
    min_dists_t2 = []
    min_dists_t4 = []
    valid_labels = []

    rho_s = 10.67 / 2  # Veiligheidsmarge (helft van de scheepsbreedte)

    for i, run_id in enumerate(run_ids):
        output_dir = f'output/run_{run_id}'
        try:
            df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')))
            df_t2 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other2.csv')))
            df_t4 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other4.csv')))
        except Exception as e:
            print(f"Skipping Run {run_id} ({labels[i]}): Data mist of incompleet.")
            continue
            
        # Bereken de afstand tot Target 2 op elk overlappend tijdstip
        merged_t2 = pd.merge_asof(df_own, df_t2, on='t', suffixes=('_own', '_tgt'))
        d2 = np.sqrt((merged_t2['x_true_own'] - merged_t2['x_true_tgt'])**2 + 
                     (merged_t2['y_true_own'] - merged_t2['y_true_tgt'])**2)
        min_dists_t2.append(d2.min())
        
        # Bereken de afstand tot Target 4 op elk overlappend tijdstip
        merged_t4 = pd.merge_asof(df_own, df_t4, on='t', suffixes=('_own', '_tgt'))
        d4 = np.sqrt((merged_t4['x_true_own'] - merged_t4['x_true_tgt'])**2 + 
                     (merged_t4['y_true_own'] - merged_t4['y_true_tgt'])**2)
        min_dists_t4.append(d4.min())
        
        valid_labels.append(labels[i])
        
    if not valid_labels:
        return

    # Print de harde getallen naar de console (handig voor in je LaTeX tekst/tabellen)
    print(f"\n--- Minimum Distances: {category_name} ---")
    print(f"{'Condition':<12} | {'Target 2 (m)':<15} | {'Target 4 (m)':<15}")
    print("-" * 50)
    for i, label in enumerate(valid_labels):
        print(f"{label:<12} | {min_dists_t2[i]:<15.2f} | {min_dists_t4[i]:<15.2f}")

    # --- Plotten van de Bar Chart ---
    x = np.arange(len(valid_labels))  # De label locaties op de x-as
    width = 0.35  # Breedte van de balkjes
    
    fig, ax = plt.subplots(figsize=(8, 6))
    rects1 = ax.bar(x - width/2, min_dists_t2, width, label='Target 2', color='forestgreen', alpha=0.8, edgecolor='black')
    rects2 = ax.bar(x + width/2, min_dists_t4, width, label='Target 4', color='mediumvioletred', alpha=0.8, edgecolor='black')
    
    # Veiligheidsmarge tekenen
    ax.axhline(rho_s, color='red', linestyle='--', linewidth=2, label=f'Collision Boundary ($\\rho_s = {rho_s:.2f}$ m)')
    
    # Opmaak
    ax.set_ylabel('Minimum Absolute Distance (m)')
    ax.set_title(f'Minimum Encounter Distance - {category_name}', fontweight='bold', pad=15)
    ax.set_xticks(x)
    ax.set_xticklabels(valid_labels)
    ax.legend(loc='upper right')
    
    # Voeg de exacte getallen toe bovenop de balkjes
    ax.bar_label(rects1, padding=3, fmt='%.1f')
    ax.bar_label(rects2, padding=3, fmt='%.1f')
    
    # Zorg dat de Y-as hoog genoeg is, zodat de text-labels niet buiten de grafiek vallen
    max_val = max(max(min_dists_t2), max(min_dists_t4))
    ax.set_ylim(0, max_val * 1.25)
    
    plt.tight_layout()
    
    # Opslaan op een Windows-veilige manier
    save_dir = os.path.join('output', 'analysis_plots')
    os.makedirs(save_dir, exist_ok=True)
    safe_name = category_name.replace(' ', '_').replace('°', 'deg')
    filename = os.path.join(save_dir, f'CPA_BarChart_{safe_name}.png')
    
    plt.savefig(filename, dpi=300)
    print(f"Plot opgeslagen als: {filename}")
    plt.close() # Voorkomt dat er 6 grafieken tegelijk op je scherm poppen, hij slaat ze netjes op!

if __name__ == '__main__':
    # Zorg dat de 'labels' lijst precies overeenkomt met de verstoringen uit je runs
    
    plot_min_distance_bar_chart([0], ['Baseline'], 'Baseline')
    
    plot_min_distance_bar_chart([1, 2, 3], ['0.1m', '5.0m', '10.0m'], 'GPS Noise')
    
    plot_min_distance_bar_chart([4, 5, 6], ['0.1°', '1.0°', '3.0°'], 'Heading Noise')
    
    plot_min_distance_bar_chart([7, 8, 9], ['5.0 m/s', '10.0 m/s', '15.0 m/s'], 'Wind (90°)')
    
    plot_min_distance_bar_chart([10, 11, 12], ['0.5 m/s', '1.0 m/s', '1.5 m/s'], 'Current (90°)')
    
    # Pas de wave labels hieronder aan naar de H_s waarden die je uiteindelijk in je simulatie gebruikt hebt (bijv. 0.5m, 1.0m, 2.0m)
    plot_min_distance_bar_chart([13, 14, 15], ['0.5m', '1.0m', '2.0m'], 'Waves (90°)')