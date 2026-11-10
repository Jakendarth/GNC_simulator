#================================================
# Description: This file is the main file of the project. 
# It imports all the necessary modules and runs the program.
#================================================
import sys
import os
from owlready2 import * 
import pygame 

current_script_dir = os.path.dirname(os.path.abspath(__file__))
path_to_src = os.path.join(current_script_dir, "src")

if path_to_src not in sys.path:
    sys.path.insert(0, path_to_src)
    sys.path.insert(0, os.path.join(path_to_src, "modules"))
    sys.path.insert(0, os.path.join(path_to_src, "pipelines"))

from src.modules.helper_utils import load_sim

#=============================================
# Modify these lines for each run
#=============================================
input_file = os.path.join(current_script_dir, "input/SimParam_WC.json")
CURRENT_RUN_ID = 9 # Change this to 2 or 3 for the next runs
# Run 0 is clean run, without noise
# Run 1, 2 and 3 correspond to different GPS noise levels (0.1m, 5m, 10m respectively).
# Run 4, 5, 6 correspond to different heading noise levels (0.1deg, 1 deg, 3 deg respecitively).
# Run 7, 8, 9 correspond to different wind speeds (5.0 m/s +-0.5m/s +-5deg, 10.0 m/s+-1, 15.0 m/s +- 1 m/s respectively) on 90 degrees with noise at 3 degrees.
#  Run 10, 11, 12 correspond to different current speeds (0.5 m/s, 1.0 m/s, 1.5 m/s) on 90 degrees with +-0.1m/s +-5deg noise.
# Run 13, 14, 15 corrsepond to different wave heights (1.0 m +-0.3, 5 m +- 1 m, 12 m +- 2) on 90 degrees
#=============================================

def main(output_dir):
    libs = load_sim(input_file)
    run_simulation = libs["run_simulation"]
    pygame.init()
    screen = pygame.display.set_mode((1200, 800))
    
    try:
        a = pygame.image.load(os.path.join(current_script_dir, "assets/safenet.png"))
        pygame.display.set_icon(a)
    except FileNotFoundError:
        pass
        
    font = pygame.font.SysFont('Consolas', 25)
    
    print(f"\n=== Start simulation run {CURRENT_RUN_ID} ===")
    
    # Maak een unieke output map aan voor deze specifieke run
    run_output_dir = os.path.join(output_dir, f"run_{CURRENT_RUN_ID}")
    os.makedirs(run_output_dir, exist_ok=True)
    
    # IMPORTANT: reload the ontology per run.
    ontology = get_ontology(os.path.join(current_script_dir, "assets/database.xml")).load()
    
    # Draai de simulatie
    run_simulation(screen, ontology, input_file, font, current_script_dir, run_output_dir)

#=============================================
# Leave this untouched
#=============================================
if (__name__=='__main__'):
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = "output"
    main(output_dir)
#=============================================