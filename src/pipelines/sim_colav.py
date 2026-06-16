import numpy as np
from plots import *
import json
from traffic_planner import PID_controller
from intel_guidance import Guidance_Reasoner, compute_LOS_ref_adaptive
from mapper import create_map
from helper_utils import read_vessels, draw_vessels, calc_bank_distances, export_simulation_data
import vidmaker
import csv
import copy
import os
                

def run_simulation(screen,ontology,file_path,font,current_script_dir,output_dir):
    """
    Simulates the navigation of an inland vessel and visualizes it using Pygame.
    Parameters:
        screen (pygame.Surface): The Pygame surface to draw the simulation on.
        vessels[0] (Vessel): An instance of the Vessel class representing the vessel being simulated.
        sim_time (float): The total simulation time.
        t_step (float): The time step for each iteration of the simulation.
    Returns:
        None
    """
    #=========================================
    # Initialize Pygame parameters
    #=========================================
    time=0
    times_to_draw=[]
    clock = pygame.time.Clock()
    running = True
    pygame.display.set_caption('WC Case study')
    with open(file_path) as f: # Configure simulation params
        d = json.load(f)
        sim_time = d['Simulation Time']
        sim_scale=d['scale']
        t_step = d['Time step']
        vessels=read_vessels(os.path.join(current_script_dir,'assets/vessel_traffic.csv'),ontology,d['Sensor noise'],current_script_dir)
    with open(os.path.join(current_script_dir,'assets/vessel_traffic.csv'), mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.reader(infile)
        next(reader)
        for row in reader:
            times_to_draw.append(float(row[7]))
    #=========================================
    # Initialize lists to store simulation data and video maker
    #=========================================
    video = vidmaker.Video(path=os.path.join(output_dir,"case_study_frame.mp4"),fps=30,late_export=True)
    #=========================================
    # Initialise other parameters
    #=========================================
    U=0
    d=0
    #=====================================================
    # Here we can set the force_switch to use the corresponding model to the navigation scenario (e.g., inland, short-sea)
    #=====================================================
    for v in vessels: # Determine force effects on vessel model
        v.ontology.force_switch = [1, 1, 1, 0]
    #=========================================
    # Run the simulation
    #=========================================
    while running and (time < sim_time) and (vessels[0].y[3]<=vessels[0].route[0][-1]):
        #=====================================================
        # Here we can set stop conditions (e.g., quit button pressed, vessel collided with boundaries)
        #=====================================================
        # print("My vessel is at x=", vessels[0].y[3])
        # print("Last point of myvessel route is x=", vessels[0].route[0][-1])
        for event in pygame.event.get(): 
            if (event.type == pygame.QUIT):
                running = False
        #=====================================================
        # GNC Implementation
        #=====================================================
        for ii in range(0, len(vessels)):
            instance=vessels[ii]
            other_vessels=[v for v in vessels if v.type!=instance.type]
            if ((instance.time>=times_to_draw[ii]) and (instance.arrived!=1)):
                #=====================================================
                # Guidance system
                #=====================================================
                Dref_route_own, U, d, enc_cond,v2=Guidance_Reasoner(time,instance, other_vessels, ontology, sim_scale,current_script_dir) # Collision-free Path Planner
                psi_d=compute_LOS_ref_adaptive(instance, Dref_route_own,current_script_dir) # LOS Guidance
                pygame.display.set_caption('WC Case study ['+vessels[0].scenario+', '+vessels[0].role+', '+v2.scenario+', '+v2.role+']')       
                #=======================================================
                # Control system
                #=======================================================
                instance.u.append(PID_controller(instance, psi_d, t_step)) # Compute the control input
                #=======================================================
                # Navigation system
                #=======================================================
                t1,y_zeta=instance.differential_algebraic_model(t_step)
                #instance.usafe.append((time,U))
                #instance.time+=t_step
            else:
                t1=np.linspace(0,t_step,40)
                y_zeta=[[instance.state[ii]]*np.shape(t1)[0] for ii in range(0,len(instance.state)) if ii!=5] 
                #instance.u.append(0)
                psi_d=instance.state[-1]
            instance.dynamic_ontology_update(copy.deepcopy(y_zeta),copy.deepcopy(psi_d),copy.deepcopy(t1),copy.deepcopy(time)) # Update the ontology with the latest states
        #=========================================
        # Draw the simulation [PyGame GUI]
        #=========================================
        x_map, y_map = create_map(screen, font,os.path.join(current_script_dir,'assets/navigation_map.csv'),vessels[0])
        draw_vessels(screen, vessels, sim_scale, d, time, times_to_draw) # Draw the vessels
        calc_bank_distances(screen, vessels, sim_scale, time, times_to_draw, x_map, y_map)
        #time=time+t_step
        #-----------------------------------------
        # Update the display
        #-----------------------------------------
        pygame.display.update()
        video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1),inverted=False)
        time=time+t_step
        clock.tick(300)  # Slower frame rate for smoother motion
    #=========================================
    # Plot the simulation data
    #=========================================
    #for l in [vessels[0].ontology.time,vessels[0].ontology.x,vessels[0].ontology.y,vessels[0].ontology.SOG,vessels[0].ontology.yx,vessels[0].ontology.yy,vessels[0].ontology.ypsi,vessels[0].ontology.psi,vessels[0].ontology.psi_d,vessels[0].ontology.rudder_angle]:
    #     l.remove(0)
    video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1),inverted=False)
    video.export(verbose=True) # save the recording (per frame)
    export_simulation_data(vessels, output_dir)
