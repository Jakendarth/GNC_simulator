import os
import pandas as pd
import matplotlib.pyplot as plt

def clean_data(df, time_col='t'):
    """Sorteert de data chronologisch en verwijdert de t=0 initialisatie."""
    df = df.sort_values(by=time_col).reset_index(drop=True)
    return df[df[time_col] > 0]

def main():
    output_dir = 'output/run_1' 
    
    # 1. Laad de bestanden van alle schepen in
    try:
        df_own = clean_data(pd.read_csv(os.path.join(output_dir, 'Own.csv')), 't')
        df_other2 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other2.csv')), 't')
        df_other4 = clean_data(pd.read_csv(os.path.join(output_dir, 'Other4.csv')), 't')
    except FileNotFoundError as e:
        print(f"Kan bestanden niet vinden: {e}. Zorg dat je 'python main.py' hebt gedraaid met de nieuwe CSV-kolommen.")
        return

    plt.figure(figsize=(12, 8))
    
    # --- Own Vessel (Blauw / Zwart) ---
    plt.scatter(df_own['x_noisy'], df_own['y_noisy'], 
                s=4, color='blue', alpha=0.3, label='GPS Own')
    plt.plot(df_own['x_true'], df_own['y_true'], 
             color='black', linewidth=1.5, label='Actual Own')

    # --- Target 2 (Groen) ---
    plt.scatter(df_other2['x_noisy'], df_other2['y_noisy'], 
                s=4, color='limegreen', alpha=0.3, label='GPS Target 2')
    plt.plot(df_other2['x_true'], df_other2['y_true'], 
             color='darkgreen', linewidth=1.5, label='Actual Target 2')

    # --- Target 4 (Paars) ---
    plt.scatter(df_other4['x_noisy'], df_other4['y_noisy'], 
                s=4, color='mediumorchid', alpha=0.3, label='GPS Target 4')
    plt.plot(df_other4['x_true'], df_other4['y_true'], 
             color='indigo', linewidth=1.5, label='Actual Target 4')

    # Startpunten markeren met een duidelijke stip
    plt.scatter(df_own['x_true'].iloc[0], df_own['y_true'].iloc[0], color='black', marker='o', s=60, zorder=5)
    plt.scatter(df_other2['x_true'].iloc[0], df_other2['y_true'].iloc[0], color='darkgreen', marker='o', s=60, zorder=5)
    plt.scatter(df_other4['x_true'].iloc[0], df_other4['y_true'].iloc[0], color='indigo', marker='o', s=60, zorder=5)

    # Opmaak in de stijl van het voorbeeld
    plt.xlabel('Position: East (m)')
    plt.ylabel('Position: North (m)')
    plt.title('Position (Actual vs GPS Noise) - Multi-Vessel')
    plt.legend(loc='best', markerscale=2) # Maakt de bolletjes in de legenda iets groter voor leesbaarheid
    plt.grid(True, alpha=0.3)
    plt.axis('equal') 
    
    # Opslaan
    plt.savefig(os.path.join(output_dir, 'Actual_vs_GPS_Multi_Trajectory.png'), dpi=300)
    plt.show()

if __name__ == '__main__':
    main()
