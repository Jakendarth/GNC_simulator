#================================================
# Description: This file is the main file of the project. 
# It imports all the necessary modules and runs the program.
#================================================
import sys
import os
import shutil
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
CURRENT_RUN_ID = 1  # Change this for the next runs
# Run 0: Clean baseline run, without noise or disturbances.
# Run 1, 2, 3: GPS noise levels (sigma = 0.1 m, 5.0 m, 10.0 m).
# Run 4, 5, 6: Heading noise levels (sigma = 0.1 deg, 1.0 deg, 3.0 deg).
# Run 7, 8, 9: Wind speeds at 90 deg (5.0 m/s +-1.0 m/s +-3 deg; 10.0 m/s +-1.0 m/s +-5 deg; 15.0 m/s +-1.0 m/s +-5 deg).
# Run 10, 11, 12: Current speeds at 90 deg (0.5 m/s, 1.0 m/s, 1.5 m/s) with isotropic sigma_c = 0.1 m/s.
# Run 13, 14, 15: Wave heights at 90 deg (0.5 m +-0.3 m, 1.0 m +-0.5 m, 2.0 m +-0.5 m).
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
    
    # Sla de exacte parameters en gebruikte seed op bij de resultaten
    shutil.copy(input_file, os.path.join(run_output_dir, "SimParam_used.json"))
    
    # IMPORTANT: reload the ontology per run.
    ontology = get_ontology(os.path.join(current_script_dir, "assets/database.xml")).load()
    
    # Draai de simulatie
    run_simulation(screen, ontology, input_file, font, current_script_dir, run_output_dir)

#=============================================
# Leave this untouched
#=============================================
if (__name__ == '__main__'):
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        output_dir = "output"
    main(output_dir)
#=============================================