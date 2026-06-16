import json
import copy
import os
import importlib 
import pygame
import csv
import pandas as pd
from classes import vessel
from intel_guidance import plan_route
import numpy as np

def load_sim(input_file):
    with open(input_file, "r") as f:
        cfg = json.load(f)
    loaded = {}

    for item in cfg.get("imports", []):

        # Case 1: import module (optionally with alias)
        if "module" in item:
            mod = importlib.import_module(item["module"])
            alias = item.get("alias", item["module"].split(".")[-1])
            loaded[alias] = mod

        # Case 2: from X import a, b, c
        elif "from" in item and "import" in item:
            mod = importlib.import_module(item["from"])
            for name in item["import"]:
                loaded[name] = getattr(mod, name)

    return loaded

def read_vessels(input_file,ontology,sensor_noise,current_script_dir):
    vessels=[]
    with open(input_file, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.reader(infile)
        next(reader)
        for row in reader:
            in_cond=[float(row[4]), float(row[5]), float(row[6]), float(row[1]),float(row[2]), 0, float(row[3])]
            name=row[8]
            instance = vessel(name, ontology, in_cond)
            instance.route=plan_route([float(row[1]),float(row[2])],[float(row[9]),float(row[10])],1,current_script_dir)
            instance.include_sensor_noise(sensor_noise)
            vessels.append(instance)
    return vessels

def get_font(size): # Returns Press-Start-2P in the desired size
    return pygame.font.Font("assets/Times_New_Roman.ttf", size)

def draw_ellipse_angle(surface, color, rect, angle, width=0):
    """
    Draws an ellipse on a given surface with a specified angle of rotation.
    Parameters:
        surface (pygame.Surface): The surface on which to draw the ellipse.
        color (tuple): The color of the ellipse, in RGB format.
        rect (tuple): A tuple representing the rectangle that bounds the ellipse (x, y, width, height).
        angle (float): The angle by which to rotate the ellipse, in degrees.
        width (int, optional): The width of the ellipse's border. Defaults to 0 for a filled ellipse.
    Returns:
        None
    """

    target_rect = pygame.Rect(rect)
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.ellipse(shape_surf, color, (0, 0, *target_rect.size), width)
    rotated_surf = pygame.transform.rotate(shape_surf, angle)
    surface.blit(rotated_surf, rotated_surf.get_rect(center = target_rect.center))

def draw_vessels(screen, players, sim_scale, safe_distance_L, time, times_to_draw):
    for ii in range(0,len(players)):
        instance=players[ii]
        time=times_to_draw[ii]
        if (time<=instance.time):
            ship_x = sim_scale*copy.deepcopy(instance.ontology.x[-1])# 1. Transform position to map
            ship_y = -sim_scale*copy.deepcopy(instance.ontology.y[-1]) 
            # 2. Draw the vessel
            draw_ellipse_angle(screen, "blue", [ship_x-sim_scale*instance.ontology.L[0]/2, ship_y-sim_scale*instance.ontology.B[0]/2, sim_scale*instance.ontology.L[0], sim_scale*instance.ontology.B[0]], instance.ontology.psi[-1]*180/np.pi)
            # 3. Draw safe distance
            pygame.draw.circle(screen, "orange", (ship_x, ship_y), sim_scale*(safe_distance_L+instance.ontology.L[0]/2), 1)
            # 4. Draw ship domain
            pygame.draw.circle(screen, "red", (ship_x, ship_y), sim_scale*instance.ontology.L[0]/2,1)
            # 5. Draw encounter radius
            pygame.draw.circle(screen, "cyan", (ship_x, ship_y), sim_scale*5*instance.ontology.L[0], 1)
            # 6. Draw the path and active segment
            draw_route(screen, sim_scale, instance)

def draw_route(screen, sim_scale,  instance):
    #for k in range(0,len(instance.route[0])-1):
    route_points=[np.array([instance.route[0][ii], instance.route[1][ii]]) for ii in range(0,len(instance.route[0]))]
    distance_from_current=[np.sqrt((route_points[jj][0] - instance.y[3])**2 + (route_points[jj][1] - instance.y[4])**2 ) for jj in range(0,len(route_points))]
    k=np.argmin(distance_from_current)
    for ii in range (0,len(instance.route[0])-1):
        pygame.draw.line(screen, "gray", (sim_scale*instance.droute[0][ii]+sim_scale*instance.route[0][ii] ,-sim_scale*instance.route[1][ii]-sim_scale*instance.droute[1][ii] ), (sim_scale*instance.droute[0][ii+1]+sim_scale*instance.route[0][ii+1] ,-sim_scale*instance.route[1][ii+1]-sim_scale*instance.droute[1][ii+1] ), 2)
    if (k<len(instance.route[0])-1):
        pygame.draw.line(screen, "red", (sim_scale*instance.droute[0][k]+sim_scale*instance.route[0][k] ,-sim_scale*instance.route[1][k]-sim_scale*instance.droute[1][k] ), (sim_scale*instance.droute[0][k+1]+sim_scale*instance.route[0][k+1] ,-sim_scale*instance.route[1][k+1]-sim_scale*instance.droute[1][k+1]), 2)
        pygame.draw.circle(screen,'black',(sim_scale*instance.droute[0][k]+sim_scale*instance.route[0][k] ,-sim_scale*instance.route[1][k]-sim_scale*instance.droute[1][k] ),5,1)
        pygame.draw.circle(screen,'black',(sim_scale*instance.droute[0][k+1]+sim_scale*instance.route[0][k+1] ,-sim_scale*instance.route[1][k+1]-sim_scale*instance.droute[1][k+1] ), 5,1)
        instance.arrived=0
    else:
        instance.arrived=1


def calc_bank_distances(screen, vessels, sim_scale, time, times_to_draw, map_x, map_y):
    #=====================================
    # 1. Configure axis where boundaries should be sought
    #=====================================
    #print(map_x, map_y)
    for ii in range(0,len(vessels)):
        v=vessels[ii]
        time=times_to_draw[ii]
        if (time<=v.time):
            COG_vessel_axes=np.array([[0], [0]])
            rot_matrix=np.array([[np.cos(v.ontology.psi[-1]),-np.sin(v.ontology.psi[-1])], [np.sin(v.ontology.psi[-1]),np.cos(v.ontology.psi[-1])]])
            COG_global_axes= np.array([[sim_scale*copy.deepcopy(v.ontology.x[-1])],[-sim_scale*copy.deepcopy(v.ontology.y[-1])]])+np.matmul(rot_matrix,COG_vessel_axes)
            ship_x=COG_global_axes[0][0]
            ship_y=COG_global_axes[1][0]
            pygame.draw.circle(screen, (0,0,0), (ship_x,ship_y),1) # Draw the center of gravity
            bank_distances=[]
            for ii in range(0,len(map_x)):
                condition_1= (COG_global_axes[0][0]<=np.max(map_x[ii])) and (COG_global_axes[0][0]>=np.min(map_x[ii])) # Check if COG passes along banks/ obstacles in x-axis
                condition_2= (COG_global_axes[1][0]<=np.max(map_y[ii])) and (COG_global_axes[1][0]>=np.min(map_y[ii])) # Check if COG passes along banks/ obstacles in y-axis
                condition_3= (np.abs(v.ontology.psi[-1])<np.pi/4) or (np.abs(v.ontology.psi[-1]-np.pi)<np.pi/4)
                condition_4= (np.abs(v.ontology.psi[-1])>=np.pi/4) or (np.abs(v.ontology.psi[-1]-np.pi)>=np.pi/4)
                condition_5=COG_global_axes[0][0]>=np.min(np.min(map_x))
                if ((condition_1) and (condition_3)):
                    jj=np.argmin(np.abs(map_y[ii]-ship_y))
                    if (map_y[ii][jj]>ship_y):
                        pygame.draw.line(screen, "green", (ship_x, ship_y), (ship_x, map_y[ii][jj]), 2)      
                        v.distance_from_bank1=np.sqrt((map_y[ii][jj]-ship_y)**2)/sim_scale                   # Distance from starboard side of canal
                    else:
                        pygame.draw.line(screen, "yellow", (ship_x, ship_y), (ship_x, map_y[ii][jj]), 2)
                        v.distance_from_bank2=-np.sqrt((map_y[ii][jj]-ship_y)**2)/sim_scale                  # Distance from port side of canal
                elif ((condition_2) and (condition_4) and (condition_5)):
                    jj=np.argmin(np.abs(map_x[ii]-ship_x))
                    if (map_x[ii][jj]>ship_x):
                        pygame.draw.line(screen, "green", (ship_x, ship_y), (map_x[ii][jj], ship_y), 2)
                        v.distance_from_bank1=np.sqrt((map_x[ii][jj]-ship_x)**2)/sim_scale
                    else:
                        pygame.draw.line(screen, "yellow", (ship_x, ship_y), (map_x[ii][jj], ship_y), 2)
                        v.distance_from_bank2=-np.sqrt((map_x[ii][jj]-ship_x)**2)/sim_scale
                else:
                    pass
                #v.distance1_vals.extend([v.distance_from_bank1]*len(t1.tolist()))
            if len(bank_distances)>1:
                v.ontology.force_switch = [1, 1, 1, 1]
            else:
                v.ontology.force_switch = [1, 1, 1, 0]
        # if (sim_scale*copy.deepcopy(vessels[0].ontology.x[-1])>=600):
        #         distance_from_bank1=pygame.draw.line(screen, "white", (ship_x-sim_scale*vessels[0].ontology.B[0]/2*np.sin(vessels[0].ontology.psi[-1]), ship_y-sim_scale*vessels[0].ontology.B[0]/2*np.cos(vessels[0].ontology.psi[-1])), (ship_x, 325), 2)
        #         #distance_from_bank1o=pygame.draw.line(screen, "white", (ship_xo1-vessels[2].ontology.B[0]/2*np.sin(psio1_vals[-1]), ship_yo1-vessels[2].ontology.B[0]/2*np.cos(psio1_vals[-1])), (ship_xo1, 200), 2)
        #         distance_from_bank2=pygame.draw.line(screen, "white", (ship_x+sim_scale*vessels[0].ontology.B[0]/2*np.sin(vessels[0].ontology.psi[-1]), ship_y+sim_scale*vessels[0].ontology.B[0]/2*np.cos(vessels[0].ontology.psi[-1])), (ship_x, 475), 2)
        #         #distance_from_bank2o=pygame.draw.line(screen, "white", (ship_xo1+vessels[2].ontology.B[0]/2*np.sin(psio1_vals[-1]), ship_yo1+vessels[2].ontology.B[0]/2*np.cos(psio1_vals[-1])), (ship_xo1, 600), 2)
        #         vessels[0].distance_from_bank1=np.sqrt((vessels[0].ontology.B[0]/2*np.sin(vessels[0].ontology.psi[-1]))**2+(ship_y/sim_scale-vessels[0].ontology.B[0]/2*np.cos(vessels[0].ontology.psi[-1])-325/sim_scale)**2)
        #     vessels[1].distance_from_bank1=np.inf
        #     try:
        #         vessels[2].distance_from_bank1=np.sqrt((vessels[2].ontology.B[0]/2*np.sin(vessels[2].ontology.psi[-1]))**2+(ship_yo2/sim_scale-vessels[2].ontology.B[0]/2*np.cos(vessels[2].ontology.psi[-1])-325/sim_scale)**2)
        #     except:
        #         pass
        #     vessels[0].distance_from_bank2=np.sqrt((vessels[0].ontology.B[0]/2*np.sin(vessels[0].ontology.psi[-1]))**2+(ship_y/sim_scale+vessels[0].ontology.B[0]/2*np.cos(vessels[0].ontology.psi[-1])-475/sim_scale)**2)
        #     vessels[1].distance_from_bank2=np.inf
        #     try:
        #         vessels[2].distance_from_bank2=np.sqrt((vessels[2].ontology.B[0]/2*np.sin(vessels[2].ontology.psi[-1]))**2+(ship_yo2/sim_scale+vessels[2].ontology.B[0]/2*np.cos(vessels[2].ontology.psi[-1])-475/sim_scale)**2)
        #     except:
        #         pass


def export_simulation_data(vessels, output_dir='out/sim'):
    """
    Export simulation data to CSV files including vessel states, sensor values, and bank distances.
    
    Parameters:
        vessels (list): List of vessel instances from the simulation
        output_dir (str): Directory to save CSV files (default: 'output')
    
    Returns:
        None
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    # Export data for each vessel
    for idx, vessel in enumerate(vessels):
        # Export combined comprehensive data
        log_data1 = {
            't': vessel.ontology.time,
            'x': vessel.ontology.yx,
            'y': vessel.ontology.yy,
            'psi': vessel.ontology.ypsi,
            'psi_d': vessel.ontology.psi_d,
            'SOG': vessel.ontology.SOG,
            'delta': vessel.ontology.rudder_angle,
            'dls': vessel.ontology.starboard_distance,
            'dlp': vessel.ontology.port_distance,

        }
        log_data2 = {
            'b':vessel.beta,
            'betahat': vessel.betahat,
            'ehat': vessel.ehat,
            #'e':vessel.e,
            'uhat':vessel.uhat,
            'psihat':vessel.psihat,
            'fhatu':vessel.fhatu,
            'fhatpsi':vessel.fhatpsi,
            'timehat':vessel.timehat

        }
        # Validate all arrays have the same length
        lengths1 = {key: len(value) for key, value in log_data1.items()}
        lengths2 = {key: len(value) for key, value in log_data2.items()}
        if len(set(lengths1.values())) > 1:
            print(f"Warning: Vessel {idx} comprehensive data has inconsistent array lengths: {lengths1}")
            # Find the minimum length to truncate all arrays
            min_length1 = min(lengths1.values())
            log_data1 = {key: value[:min_length1] for key, value in log_data1.items()}
        if len(set(lengths2.values())) > 1:
            print(f"Warning: Vessel {idx} comprehensive data has inconsistent array lengths: {lengths2}")
            # Find the minimum length to truncate all arrays
            min_length2 = min(lengths2.values())
            log_data1 = {key: value[:min_length2] for key, value in log_data1.items()}
        df1_comprehensive = pd.DataFrame(log_data1)
        df2_comprehensive = pd.DataFrame(log_data2)
        comprehensive_filename1 = f'{output_dir}/{vessel.type}.csv'
        comprehensive_filename2 = f'{output_dir}/{vessel.type}_estimation.csv'
        df1_comprehensive.to_csv(comprehensive_filename1, index=False)
        df2_comprehensive.to_csv(comprehensive_filename2, index=False)
    
    print(f"Simulation data exported to {output_dir}/ directory")