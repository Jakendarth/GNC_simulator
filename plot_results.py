#!/usr/bin/env python3
"""
Plot simulation results from vessel comprehensive CSV files.
Generates:
1. X-Y trajectory plot for all vessels
2. Time vs heading (psi) plot for all vessels
"""

import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Set style for publication-quality plots
plt.style.use('seaborn-v0_8-darkgrid')
plt.rcParams['figure.figsize'] = (12, 8)
plt.rcParams['figure.dpi'] = 100
plt.rcParams['font.size'] = 12
plt.rcParams['axes.labelsize'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['legend.fontsize'] = 10

def main():
    # Find all vessel comprehensive CSV files
    output_dir = 'output'
    vessel_files = glob.glob(os.path.join(output_dir, 'vessel_*_comprehensive.csv'))
    
    if not vessel_files:
        print("No vessel comprehensive CSV files found in output/ directory")
        return
    
    # Load data for each vessel
    vessels_data = {}
    for file in vessel_files:
        # Extract vessel name from filename
        filename = os.path.basename(file)
        # Format: vessel_{index}_{type}_comprehensive.csv
        parts = filename.split('_')
        if len(parts) >= 3:
            vessel_type = parts[1]  # e.g., 'Own', 'Other1', etc.
            df = pd.read_csv(file)
            vessels_data[vessel_type] = df
            print(f"Loaded {vessel_type}: {len(df)} data points")
    
    print(f"\nTotal vessels loaded: {len(vessels_data)}")
    
    # Define colors for different vessels
    colors = plt.cm.tab10(np.linspace(0, 1, len(vessels_data)))
    
    # =========================================================================
    # Plot 1: X-Y Trajectories
    # =========================================================================
    print("\nGenerating trajectory plot...")
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Plot trajectory for each vessel
    for idx, (vessel_type, df) in enumerate(vessels_data.items()):
        ax.plot(df['x'], df['y'], 
                linewidth=2, 
                color=colors[idx],
                label=vessel_type,
                alpha=0.8)
        
        # Mark start and end points
        ax.scatter(df['x'].iloc[0], df['y'].iloc[0], 
                    marker='o', s=100, color=colors[idx], 
                    edgecolors='black', linewidths=2, zorder=5)
        ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], 
                    marker='s', s=100, color=colors[idx], 
                    edgecolors='black', linewidths=2, zorder=5)
    
    # Customize the plot
    ax.set_xlabel('X Position (m)', fontsize=14)
    ax.set_ylabel('Y Position (m)', fontsize=14)
    ax.set_title('Vessel Trajectories', fontsize=16, fontweight='bold')
    ax.legend(loc='best', frameon=True, shadow=True)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')  # Equal aspect ratio for proper spatial representation
    
    # Add legend annotations
    ax.text(0.02, 0.98, '○ = Start', transform=ax.transAxes, 
            fontsize=10, verticalalignment='top', 
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    ax.text(0.02, 0.93, '□ = End', transform=ax.transAxes, 
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    
    # Save figure
    fig.savefig('output/vessel_trajectories.png', dpi=300, bbox_inches='tight')
    print(f"✓ Trajectory plot saved to: output/vessel_trajectories.png")
    
    # =========================================================================
    # Plot 2: Time vs Heading (Psi)
    # =========================================================================
    print("\nGenerating heading plot...")
    fig, ax = plt.subplots(figsize=(14, 6))
    
    # Plot heading for each vessel
    for idx, (vessel_type, df) in enumerate(vessels_data.items()):
        ax.plot(df['t'], df['psi'], 
                linewidth=2, 
                color=colors[idx],
                label=f'{vessel_type} (actual)',
                alpha=0.8)
        
        # Plot desired heading as dashed line
        ax.plot(df['t'], df['psi_d'], 
                linewidth=1.5, 
                color=colors[idx],
                linestyle='--',
                label=f'{vessel_type} (desired)',
                alpha=0.5)
    
    # Customize the plot
    ax.set_xlabel('Time (s)', fontsize=14)
    ax.set_ylabel('Heading ψ (rad)', fontsize=14)
    ax.set_title('Vessel Headings Over Time', fontsize=16, fontweight='bold')
    ax.legend(loc='best', frameon=True, shadow=True, ncol=2)
    ax.grid(True, alpha=0.3)
    
    # Add horizontal line at y=0 for reference
    ax.axhline(y=0, color='gray', linestyle=':', linewidth=1, alpha=0.5)
    
    plt.tight_layout()
    
    # Save figure
    fig.savefig('output/vessel_headings.png', dpi=300, bbox_inches='tight')
    print(f"✓ Heading plot saved to: output/vessel_headings.png")
    
    # =========================================================================
    # Summary Statistics
    # =========================================================================
    print("\n" + "="*80)
    print("VESSEL TRAJECTORY SUMMARY")
    print("="*80)
    
    for vessel_type, df in vessels_data.items():
        print(f"\n{vessel_type}:")
        print(f"  Trajectory length: {len(df)} points")
        print(f"  Time range: {df['t'].min():.2f} to {df['t'].max():.2f} s")
        print(f"  X range: {df['x'].min():.2f} to {df['x'].max():.2f} m")
        print(f"  Y range: {df['y'].min():.2f} to {df['y'].max():.2f} m")
        print(f"  Heading range: {df['psi'].min():.4f} to {df['psi'].max():.4f} rad")
        print(f"  Average SOG: {df['SOG'].mean():.2f} m/s")
        
        # Calculate total distance traveled
        dx = np.diff(df['x'])
        dy = np.diff(df['y'])
        distance = np.sum(np.sqrt(dx**2 + dy**2))
        print(f"  Total distance: {distance:.2f} m")
    
    print("\n" + "="*80)
    print("\n✓ All plots generated successfully!")
    print("  - output/vessel_trajectories.png")
    print("  - output/vessel_headings.png")

if __name__ == '__main__':
    main()
