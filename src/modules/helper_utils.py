import json
import copy
import os
import importlib 
import pygame
import csv
import pandas as pd
from shapely.geometry import box, Point, LineString
from classes import vessel
from intel_guidance import plan_route
import numpy as np
from scipy.spatial import KDTree

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

def read_vessels(input_file,ontology,sensor_noise,current_script_dir,*args):
    vessels=[]
    if len(args)==0:
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
    else:
        num_vessels=0
        in_cond=[]
        # print(args[0])
        with open(args[0], mode='r', newline='', encoding='utf-8') as infile:
            print("File read")
            reader = csv.reader(infile)
            rows=list(reader)
            in_cond=[float(x) for x in rows[1]]
            # print(in_cond)
            num_vessels=int(len(in_cond)/2)
        # print("Number of vessels:",num_vessels)
        for ii in range(num_vessels):
            st=[2,0,0,in_cond[2*ii],in_cond[2*ii+1],0,0]
            if (ii==1):
                vessels.append(vessel("Other1",ontology,st))
            else:
                vessels.append(vessel("Other2",ontology,st))
        for ii in range(num_vessels):
            vessels[ii].route=[[],[]]
            with open(args[0], mode='r', newline='', encoding='utf-8') as infile:
                reader = csv.reader(infile)
                next(reader)
                for row in reader:
                    vessels[ii].route[0].append(float(row[2*ii]))
                    vessels[ii].route[1].append(float(row[2*ii+1]))
    return vessels

def get_font(size): # Returns Press-Start-2P in the desired size
    return pygame.font.Font("assets/Times_New_Roman.ttf", size)


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

def calc_bank_distances_geopandas(screen, vessels, sim_scale, time, times_to_draw, baltic_data, *args, max_range_m=150000):
    """
    Adapted to calculate true perpendicular cross-track distances to any complex coastline
    using GeoPandas/Shapely vector maps instead of raw 1D arrays.
    """
    # 1. Collapse all landmass boundaries into a single network for lightning-fast lookups
    coastline_geom = baltic_data.geometry.unary_union.boundary
    if (args[0][0]==0):
        for ii in range(0, len(vessels)):
            v = vessels[ii]
            v_time = times_to_draw[ii]
            
            if v_time <= v.time:
                # Current heading/yaw angle from your simulation state
                psi_rad = v.ontology.psi[-1]
                
                # --- Get Global Positions (Mercator Meters) ---
                # Assuming your ontology stores positions in meters
                ship_x_m = v.ontology.x[-1]
                ship_y_m = v.ontology.y[-1]
                
                # Convert to Screen Coordinates for Pygame Drawing
                ship_px, ship_py = to_screen(ship_x_m, ship_y_m, *args)
                pygame.draw.circle(screen, (0, 0, 0), (ship_px, ship_py), 2) # Draw Center of Gravity

                # --- Construct Perpendicular Ray Vectors (Meters) ---
                vessel_point = Point(ship_x_m, ship_y_m)
                
                # Port side ray (+90 deg) & Starboard side ray (-90 deg)
                port_angle = psi_rad + (np.pi / 2)
                stbd_angle = psi_rad - (np.pi / 2)
                
                port_ray = LineString([vessel_point, (ship_x_m + max_range_m * np.cos(port_angle), 
                                                    ship_y_m + max_range_m * np.sin(port_angle))])
                stbd_ray = LineString([vessel_point, (ship_x_m + max_range_m * np.cos(stbd_angle), 
                                                    ship_y_m + max_range_m * np.sin(stbd_angle))])

                # --- Intersect and Assign Distances ---
                # Initialize safely
                v.distance_from_bank1 = max_range_m / sim_scale # Default fallback
                v.distance_from_bank2 = -max_range_m / sim_scale
                hit_counts = 0

                # Process Port Side (Corresponds to your Yellow lines / Bank 2)
                port_intersect = port_ray.intersection(coastline_geom)
                if not port_intersect.is_empty:
                    # Find the closest intersection if multiple points exist
                    pts = [p for p in port_intersect.geoms if p.geom_type == 'Point'] if port_intersect.geom_type in ['MultiPoint', 'GeometryCollection'] else [port_intersect]
                    if pts:
                        closest_pt = min(pts, key=lambda p: vessel_point.distance(p))
                        dist_m = vessel_point.distance(closest_pt)
                        
                        # Store real-world distance scaled back to model units
                        v.distance_from_bank2 = -dist_m / sim_scale 
                        hit_counts += 1
                        
                        # Visualise on Pygame
                        hit_px, hit_py = to_screen(closest_pt.x, closest_pt.y, *args)
                        pygame.draw.line(screen, "yellow", (ship_px, ship_py), (hit_px, hit_py), 2)

                # Process Starboard Side 
                stbd_intersect = stbd_ray.intersection(coastline_geom)
                if not stbd_intersect.is_empty:
                    pts = [p for p in stbd_intersect.geoms if p.geom_type == 'Point'] if stbd_intersect.geom_type in ['MultiPoint', 'GeometryCollection'] else [stbd_intersect]
                    if pts:
                        closest_pt = min(pts, key=lambda p: vessel_point.distance(p))
                        dist_m = vessel_point.distance(closest_pt)
                        
                        v.distance_from_bank1 = dist_m / sim_scale
                        hit_counts += 1
                        
                        # Visualise on Pygame
                        hit_px, hit_py = to_screen(closest_pt.x, closest_pt.y, *args)
                        pygame.draw.line(screen, "green", (ship_px, ship_py), (hit_px, hit_py), 2)
            else:
                for ii in range(0, len(vessels)):
                    v = vessels[ii]
                    # Current heading/yaw angle from your simulation state
                    psi_rad = v.ontology.psi[-1]
                        
                    # --- Get Global Positions (Mercator Meters) ---
                    # Assuming your ontology stores positions in meters
                    ship_x_m = v.ontology.x[-1]
                    ship_y_m = v.ontology.y[-1]
                        
                    # Convert to Screen Coordinates for Pygame Drawing
                    ship_px, ship_py = to_screen(ship_x_m, ship_y_m, args)
                    pygame.draw.circle(screen, (0, 0, 0), (ship_px, ship_py), 2) # Draw Center of Gravity

                    # --- Construct Perpendicular Ray Vectors (Meters) ---
                    vessel_point = Point(ship_x_m, ship_y_m)
                        
                    # Port side ray (+90 deg) & Starboard side ray (-90 deg)
                    port_angle = psi_rad + (np.pi / 2)
                    stbd_angle = psi_rad - (np.pi / 2)
                        
                    port_ray = LineString([vessel_point, (ship_x_m + max_range_m * np.cos(port_angle), 
                                                            ship_y_m + max_range_m * np.sin(port_angle))])
                    stbd_ray = LineString([vessel_point, (ship_x_m + max_range_m * np.cos(stbd_angle), 
                                                            ship_y_m + max_range_m * np.sin(stbd_angle))])

                    # --- Intersect and Assign Distances ---
                    # Initialize safely
                    v.distance_from_bank1 = max_range_m / sim_scale # Default fallback
                    v.distance_from_bank2 = -max_range_m / sim_scale
                    hit_counts = 0

                    # Process Port Side (Corresponds to your Yellow lines / Bank 2)
                    port_intersect = port_ray.intersection(coastline_geom)
                    if not port_intersect.is_empty:
                        # Find the closest intersection if multiple points exist
                        pts = [p for p in port_intersect.geoms if p.geom_type == 'Point'] if port_intersect.geom_type in ['MultiPoint', 'GeometryCollection'] else [port_intersect]
                        if pts:
                            closest_pt = min(pts, key=lambda p: vessel_point.distance(p))
                            dist_m = vessel_point.distance(closest_pt)
                                
                        # Store real-world distance scaled back to model units
                        v.distance_from_bank2 = -dist_m / sim_scale 
                        hit_counts += 1
                                
                        # Visualise on Pygame
                        hit_px, hit_py = to_screen(closest_pt.x, closest_pt.y, *args)
                        pygame.draw.line(screen, "yellow", (ship_px, ship_py), (hit_px, hit_py), 2)

                        # Process Starboard Side 
                        stbd_intersect = stbd_ray.intersection(coastline_geom)
                        if not stbd_intersect.is_empty:
                            pts = [p for p in stbd_intersect.geoms if p.geom_type == 'Point'] if stbd_intersect.geom_type in ['MultiPoint', 'GeometryCollection'] else [stbd_intersect]
                            if pts:
                                closest_pt = min(pts, key=lambda p: vessel_point.distance(p))
                                dist_m = vessel_point.distance(closest_pt)
                                
                                v.distance_from_bank1 = dist_m / sim_scale
                                hit_counts += 1
                                
                                # Visualise on Pygame
                                hit_px, hit_py = to_screen(closest_pt.x, closest_pt.y, *args)
                                pygame.draw.line(screen, "green", (ship_px, ship_py), (hit_px, hit_py), 2)
            # --- Update Force Logic Switch (Kept from your original logic) ---
            if hit_counts >= 2:
                v.ontology.force_switch = [1, 1, 1, 1]
            else:
                v.ontology.force_switch = [1, 1, 1, 0]

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


def update_ontology_start_values(player,x0):
    player.ontology.yx[0]=float(x0[3])
    player.ontology.yy[0]=float(x0[4])
    player.ontology.ypsi[0]=float(x0[-1])
    player.ontology.x[0]=float(x0[3])
    player.ontology.y[0]=float(x0[4])
    player.ontology.psi[0]=float(x0[-1])
    player.ontology.u[0] = float(x0[0])
    player.ontology.v[0] = float(x0[1])
    player.ontology.r[0] = float(x0[2])
    player.state = copy.deepcopy(x0)
    return player

def build_segment_tree(polygon_points):
    segments = []
    midpoints = []
    num_points = len(polygon_points)
    
    for j in range(num_points):
        p1 = np.array(polygon_points[j])
        p2 = np.array(polygon_points[(j + 1) % num_points])
        
        segments.append((p1, p2))
        # Calculate the midpoint of the segment to represent its location in the tree
        midpoints.append((p1 + p2) / 2.0)
        
    # Build the searchable tree
    tree = KDTree(midpoints)
    return tree, segments

def closest_point_on_segment(p, a, b):
    # (Keep the identical math from the previous response here)
    ab = b - a
    ap = p - a
    if np.all(ab == 0): return a
    t = max(0.0, min(1.0, np.dot(ap, ab) / np.dot(ab, ab)))
    return a + t * ab


def check_vessel_bounds(screen, vessels, sim_scale, times_to_draw, water_path):
    for ii in range(len(vessels)):
        v = vessels[ii]
        time = times_to_draw[ii]
        
        if time <= v.time:
            # 1. Calculate Vessel Center of Gravity (COG)
            COG_vessel_axes = np.array([[0], [0]])
            psi = v.ontology.psi[-1]
            rot_matrix = np.array([
                [np.cos(psi), -np.sin(psi)], 
                [np.sin(psi), np.cos(psi)]
            ])
            COG_global_axes = np.array([
                [sim_scale * copy.deepcopy(v.ontology.x[-1])],
                [-sim_scale * copy.deepcopy(v.ontology.y[-1])]
            ]) + np.matmul(rot_matrix, COG_vessel_axes)
            
            ship_x = COG_global_axes[0][0]
            ship_y = COG_global_axes[1][0]
            
            # 2. Check if the ship is inside the KML polygon
            is_inside = water_path.contains_point((ship_x, ship_y))
            
            # 3. Visualize a red alert if it goes out of bounds
            if not is_inside:
                ship_pos = (int(ship_x), int(ship_y))
                # Draw a thick red warning ring around the vessel
                pygame.draw.circle(screen, "red", ship_pos, 15, 3)
                # Draw a solid red dot on top of the ship
                pygame.draw.circle(screen, "red", ship_pos, 4)

def ray_segment_intersection(ray_origin, ray_dir, p1, p2):
    """
    Finds the intersection point of a ray and a line segment.
    Returns the distance 't' along the ray, and the intersection point.
    Returns (inf, None) if no valid intersection is found.
    """
    O = ray_origin
    D = ray_dir
    A = p1
    B = p2
    V = B - A

    # 2D Cross product equivalent
    cross_V_D = V[0] * D[1] - V[1] * D[0]

    # If the cross product is close to zero, the ray and segment are parallel
    if abs(cross_V_D) < 1e-6:
        return float('inf'), None

    v1 = O - A
    
    # FIXED: The cross product for 't' was reversed in the previous version, 
    # causing the distance to calculate backwards.
    cross_v1_V = v1[0] * V[1] - v1[1] * V[0] 
    cross_v1_D = v1[0] * D[1] - v1[1] * D[0]

    t = cross_v1_V / cross_V_D
    u = cross_v1_D / cross_V_D

    # t must be >= 0 (meaning the intersection is forward along the ray's direction)
    # u must be between 0 and 1 (meaning the intersection lands strictly on the segment)
    if t >= 0 and 0 <= u <= 1:
        intersect_pt = O + t * D
        return t, intersect_pt

    return float('inf'), None


def calc_bank_distances_kml(screen, vessels, sim_scale, times_to_draw, segments):
    # Note: 'tree' and 'k_nearest' have been removed from the arguments
    
    for ii in range(len(vessels)):
        v = vessels[ii]
        time = times_to_draw[ii]
        
        if time <= v.time:
            # 1. Calculate Vessel Center of Gravity (COG)
            COG_vessel_axes = np.array([[0], [0]])
            psi = v.ontology.psi[-1]
            rot_matrix = np.array([
                [np.cos(psi), -np.sin(psi)], 
                [np.sin(psi), np.cos(psi)]
            ])
            COG_global_axes = np.array([
                [sim_scale * copy.deepcopy(v.ontology.x[-1])],
                [-sim_scale * copy.deepcopy(v.ontology.y[-1])]
            ]) + np.matmul(rot_matrix, COG_vessel_axes)
            
            ship_x = COG_global_axes[0][0]
            ship_y = COG_global_axes[1][0]
            ship_pos = np.array([ship_x, ship_y])
            
            pygame.draw.circle(screen, (0, 0, 0), (int(ship_x), int(ship_y)), 1)
            
            # 2. Define Ray Directions (Port and Starboard)
            heading_x = np.cos(psi)
            heading_y = -np.sin(psi)
            
            # Starboard (90 degrees right of heading)
            ray_dir_stbd = np.array([-heading_y, heading_x])
            # Port (90 degrees left of heading)
            ray_dir_port = np.array([heading_y, -heading_x])
            
            # If you actually wanted absolute Screen UP/DOWN instead of Port/Starboard, 
            # uncomment these two lines instead:
            # ray_dir_stbd = np.array([0, -1]) # Screen UP
            # ray_dir_port = np.array([0, 1])  # Screen DOWN
            
            min_dist_stbd = float('inf')
            closest_pt_stbd = None
            
            min_dist_port = float('inf')
            closest_pt_port = None
            
            # 3. Iterate through ALL segments to ensure the ray doesn't skip the actual boundary
            for p1, p2 in segments:
                
                # Check intersection going Starboard
                t_stbd, pt_stbd = ray_segment_intersection(ship_pos, ray_dir_stbd, p1, p2)
                if t_stbd < min_dist_stbd:
                    min_dist_stbd = t_stbd
                    closest_pt_stbd = pt_stbd
                    
                # Check intersection going Port
                t_port, pt_port = ray_segment_intersection(ship_pos, ray_dir_port, p1, p2)
                if t_port < min_dist_port:
                    min_dist_port = t_port
                    closest_pt_port = pt_port
            
            # 4. Draw and assign the values for both sides
            start_pos = (int(ship_pos[0]), int(ship_pos[1]))
            
            # Starboard Update (Green)
            if closest_pt_stbd is not None:
                v.distance_from_bank1 = min_dist_stbd / sim_scale
                end_pos_stbd = (int(closest_pt_stbd[0]), int(closest_pt_stbd[1]))
                #pygame.draw.circle(screen, "red", end_pos_stbd, 6)
                #pygame.draw.line(screen, "green", start_pos, end_pos_stbd, 4)
            else:
                v.distance_from_bank1 = float('inf') 
                
            # Port Update (Yellow)
            if closest_pt_port is not None:
                v.distance_from_bank2 = -min_dist_port / sim_scale
                end_pos_port = (int(closest_pt_port[0]), int(closest_pt_port[1]))
                #pygame.draw.circle(screen, "red", end_pos_port, 6)
                #pygame.draw.line(screen, "yellow", start_pos, end_pos_port, 4)
            else:
                v.distance_from_bank2 = float('-inf')

######################################################################

def calc_bank_distances_shore(screen,
                            vessels,
                            sim_scale,
                            times_to_draw,
                            shore_top,
                            shore_bottom):

    # Build bank segments
    segments = []

    for i in range(len(shore_top) - 1):
        segments.append(
            (np.array(shore_top[i]),
             np.array(shore_top[i + 1]))
        )

    for i in range(len(shore_bottom) - 1):
        segments.append(
            (np.array(shore_bottom[i]),
             np.array(shore_bottom[i + 1]))
        )

    for ii, v in enumerate(vessels):

        time = times_to_draw[ii]

        if time > v.time:
            continue

        psi = v.ontology.psi[-1]

        ship_x = sim_scale * v.ontology.x[-1]
        ship_y = -sim_scale * v.ontology.y[-1]

        ship_pos = np.array([ship_x, ship_y])

        pygame.draw.circle(
            screen,
            (0, 0, 0),
            (int(ship_x), int(ship_y)),
            1
        )

        heading_x = np.cos(psi)
        heading_y = -np.sin(psi)

        # starboard
        ray_dir_stbd = np.array([
            -heading_y,
            heading_x
        ])

        # port
        ray_dir_port = np.array([
            heading_y,
            -heading_x
        ])

        min_dist_stbd = float('inf')
        min_dist_port = float('inf')

        closest_pt_stbd = None
        closest_pt_port = None

        for p1, p2 in segments:

            t_stbd, pt_stbd = ray_segment_intersection(
                ship_pos,
                ray_dir_stbd,
                p1,
                p2
            )

            if t_stbd < min_dist_stbd:
                min_dist_stbd = t_stbd
                closest_pt_stbd = pt_stbd

            t_port, pt_port = ray_segment_intersection(
                ship_pos,
                ray_dir_port,
                p1,
                p2
            )

            if t_port < min_dist_port:
                min_dist_port = t_port
                closest_pt_port = pt_port

        if closest_pt_stbd is not None:

            v.distance_from_bank1 = min_dist_stbd / sim_scale

            pygame.draw.line(
                screen,
                "green",
                (int(ship_x), int(ship_y)),
                (int(closest_pt_stbd[0]), int(closest_pt_stbd[1])),
                2
            )

        else:

            v.distance_from_bank1 = float('inf')

        if closest_pt_port is not None:

            v.distance_from_bank2 = -min_dist_port / sim_scale

            pygame.draw.line(
                screen,
                "yellow",
                (int(ship_x), int(ship_y)),
                (int(closest_pt_port[0]), int(closest_pt_port[1])),
                2
            )

        else:

            v.distance_from_bank2 = float('-inf')

def read_vessels_CSV_waypoints(traffic_config_file, waypoint_file, ontology, sensor_noise, current_script_dir, x_offset=0.0, y_offset=0.0):
    import numpy as np # Ensure this is imported at the top of your file
    import csv

    # 1. Parse the waypoints CSV
    vessel_routes = {}
    with open(waypoint_file, mode='r', newline='', encoding='utf-8') as wp_file:
        wp_reader = csv.DictReader(wp_file)
        for row in wp_reader:
            # Clean the mmsi string just in case
            mmsi = str(row['mmsi']).strip() 
            if mmsi not in vessel_routes:
                vessel_routes[mmsi] = ([], [])
            # Apply the offsets directly to the waypoint coordinates
            vessel_routes[mmsi][0].append(float(row['x']) + x_offset)
            vessel_routes[mmsi][1].append(float(row['y']) + y_offset)

    # 2. Read the Vessel Configuration CSV
    vessels = []
    with open(traffic_config_file, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.DictReader(infile)
        for row in reader:
            # Capture and shift the start coordinates
            vessel_x = float(row['x']) + x_offset
            vessel_y = float(row['y']) + y_offset
            name = row['name']
            
            # The shifted vessel_x and vessel_y are now passed into the initial conditions
            in_cond = [float(row['u']), float(row['v']), float(row['r']), vessel_x, vessel_y, 0, float(row['heading'])]
            instance = vessel(name, ontology, in_cond)
            instance.calculate_hydro_params()
            # 3. DYNAMIC MATCHING: Find the route that starts exactly where this vessel spawns
            best_mmsi = None
            min_dist = float('inf')
            
            for mmsi, (x_list, y_list) in vessel_routes.items():
                if len(x_list) > 0:
                    # Calculate the distance between vessel spawn and the route's first point
                    dist = np.sqrt((x_list[0] - vessel_x)**2 + (y_list[0] - vessel_y)**2)
                    
                    if dist < min_dist:
                        min_dist = dist
                        best_mmsi = mmsi
            
            # 4. Assign the matched route (allowing a 5-meter tolerance for decimal rounding differences)
            if best_mmsi is not None and min_dist < 5.0:
                instance.route = np.array([vessel_routes[best_mmsi][0], vessel_routes[best_mmsi][1]])
                print(f"Successfully linked '{name}' to route MMSI: {best_mmsi} (Start coord difference: {min_dist:.2f}m)")

                # ==========================================================
                # NEW HEADING ALIGNMENT LOGIC
                # ==========================================================
                # If the route has at least two points, point the vessel at the second point
                if len(instance.route[0]) > 1:
                    wp0_x, wp0_y = instance.route[0][0], instance.route[1][0]
                    wp1_x, wp1_y = instance.route[0][1], instance.route[1][1]
                    
                    aligned_heading = np.arctan2(wp1_y - wp0_y, wp1_x - wp0_x)
                    
                    # Apply to instance.state 
                    # Note: based on your in_cond = [u, v, r, x, y, 0, heading], heading is index 6.
                    if len(instance.state) > 6:
                        instance.state[6] = aligned_heading
                    
                    # Apply to extended_state (your compute_LOS function pulls psi from index 5 here)
                    if hasattr(instance, 'extended_state') and len(instance.extended_state) > 5:
                        instance.extended_state[5] = aligned_heading
                # ==========================================================

            else:
                print(f"WARNING: Could not find a matching route for '{name}'! Falling back to 0,0.")
                instance.route = np.array([[0], [0]])
            
            instance.include_sensor_noise(sensor_noise)
            vessels.append(instance)
            
    return vessels

def ontology_input_sync(ontology, traffic_config_file):
    with open(traffic_config_file, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.DictReader(infile)
        for row in reader:
            name = row['name']
            
            onto_instance = getattr(ontology, name, None)
            
            if onto_instance:
                # Direct mappings - wrapped in brackets for owlready2 list assignment
                if hasattr(onto_instance, 'x'):
                    onto_instance.x = [float(row['x'])]
                if hasattr(onto_instance, 'y'):
                    onto_instance.y = [float(row['y'])]
                if hasattr(onto_instance, 'u'):
                    onto_instance.u = [float(row['u'])]
                if hasattr(onto_instance, 'v'):
                    onto_instance.v = [float(row['v'])]
                if hasattr(onto_instance, 'r'):
                    onto_instance.r = [float(row['r'])]
                    
                # Cross-mappings: CSV uses 'heading', Ontology uses 'psi'
                if hasattr(onto_instance, 'psi'):
                    onto_instance.psi = [float(row['heading'])]
                
                # Optional mappings: Map spawn_time to time if appropriate
                if hasattr(onto_instance, 'time') and 'spawn_time' in row:
                    onto_instance.time = [float(row['spawn_time'])]
                
                # ==========================================
                # NEW: Dump all ontology properties 
                # ==========================================
                print(f"\n--- Full Ontology Dump for {name} ---")
                for prop in onto_instance.get_properties():
                    # prop[onto_instance] retrieves the value(s) of that property for this instance
                    value = prop[onto_instance]
                    print(f"{prop.python_name}: {value}")
                print("-------------------------------------")