#================================================
# Description: This file is the main file of the project. 
# It imports all the necessary modules and runs the program.
#================================================
import sys
# sys.path.insert(1, 'src/')
# sys.path.insert(2, 'src/modules/')
# sys.path.insert(2, 'src/pipelines/')
from owlready2 import * 
import pygame 

current_script_dir = os.path.dirname(os.path.abspath(__file__))
path_to_src = os.path.join(current_script_dir, "src")

if path_to_src not in sys.path:
    sys.path.insert(0, path_to_src)
    sys.path.insert(0, os.path.join(path_to_src, "modules"))
    sys.path.insert(0, os.path.join(path_to_src, "pipelines"))

from helper_utils import load_sim
#=============================================
# Modify only this line
#=============================================
input_file=os.path.join(current_script_dir, "input/SimParam_WC.json")
#=============================================
def main(output_dir):
    libs=load_sim(input_file)
    run_simulation=libs["run_simulation"]
    pygame.init()
    screen = pygame.display.set_mode((1200, 800))
    a = pygame.image.load(os.path.join(current_script_dir, "assets/safenet.png"))
    pygame.display.set_icon(a)
    ontology = get_ontology(os.path.join(current_script_dir,"assets/database.xml")).load()
    font = pygame.font.SysFont('Consolas', 25)
    run_simulation(screen, ontology, input_file,font,current_script_dir,output_dir)
#=============================================
# Leave this untouched
#=============================================
if (__name__=='__main__'):
    if len(sys.argv) > 1:
        output_dir = sys.argv[1]
    else:
        # Fallback default if no arg is provided
        output_dir = "output"
    main(output_dir)
#=============================================