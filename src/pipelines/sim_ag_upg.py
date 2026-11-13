from modules.classes import vessel
from modules.plots import *
import json
from modules.traffic_planner import *
from modules.channel import get_channel_depth
from modules.mapper import create_inland_depth_varying_map, draw_vessels_no_map
import vidmaker
import csv
import copy
from modules.intel_guidance import compute_LOS_ref
from modules.helper_utils import update_ontology_start_values, calc_bank_distances_shore

def run_simulation(screen, ontology, input_file, font, current_script_dir, output_dir):
    #=========================================
    # Initialize Pygame parameters
    #=========================================
    clock = pygame.time.Clock()
    pygame.display.set_caption('Anti-grounding Control')
    with open(input_file) as f: # Configure simulation params
        d = json.load(f)
        sim_time = d['Simulation Time']
        sim_scale = d['scale']
        t_step = d['Time step']
        look_ahead = d['Lookahead distance']
    d = 0
    # Constructing route
    X_NORM = np.linspace(0, 1, 20)

    CENTER_Y_NORM = (
        0.5
        + 0.2 * np.sin(1*X_NORM * np.pi)
    )

    WIDTH_NORM = np.full_like(
        X_NORM,
        0.16
    )
    # X_NORM = np.linspace(0, 1, 100)
    # CENTER_Y_NORM = 0.5 + 0.12 * np.sin(X_NORM * 2 * np.pi) + 0.04 * np.sin(X_NORM * 5.5 * np.pi)
    #WIDTH_NORM    = 0.16 + 0.04 * np.cos(X_NORM * 3 * np.pi)
    route_x = np.linspace(0, 1200, 20)
    route_y = []
    for x in route_x:
            norm_x = x / 1200
            w_portion = 0
            y_c = np.interp(norm_x, X_NORM, CENTER_Y_NORM) * 800
            route_y.append(int(max(0, y_c + w_portion)))
    route_x = [x / sim_scale for x in route_x]
    route_y = [-y / sim_scale for y in route_y]

    # Convert to lists if required elsewhere
    #route_x = route_x.tolist()
    #route_y = route_y.tolist()
    
    # Reference route arrays for tracking cost
    x_ref_route = route_x
    y_ref_route = route_y
    weights = np.array([1, 1e-3, -3e-6, 1e-7, 1e-7])
    video = vidmaker.Video(path=os.path.join(output_dir, "grounding_cs_video.mp4"), fps=30, late_export=True)
    running = True
    time = 0

    # Initialize vessels fresh for each alpha run
    in_cond = [2.5, 0, 0, route_x[0], route_y[0], 0, -0.4]
    times_to_draw = [0]
    
    barge = vessel("Own", ontology, copy.deepcopy(in_cond))
    #sbarge = vessel("Other4", ontology, copy.deepcopy(in_cond))
    barge = update_ontology_start_values(barge, copy.deepcopy(in_cond))
    barge.distance_from_bank1=24.4
    barge.distance_from_bank2=24.4
    #sbarge = update_ontology_start_values(sbarge, copy.deepcopy(in_cond))
    barge.psi_d_previous=-0.4
    barge.route = np.array([route_x, route_y])
    barge.ontology.force_switch = [1, 1, 1, 1]
    #sbarge.route = np.array([route_x, route_y])
    #sbarge.ontology.force_switch = [1, 1, 1, 0]

    shore_top, shore_bottom = create_inland_depth_varying_map(screen, barge, font, sim_scale, min_depth=3.15, max_depth=3.3)
    calc_bank_distances_shore(screen, [barge], sim_scale, times_to_draw, shore_top, shore_bottom)

    while running and (time < sim_time) and (barge.state[3] < barge.route[0][-1]):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running=False

        # Guidance system
        psi_d0 = compute_LOS_ref(barge, None, current_script_dir)
        #psi_d1 = compute_LOS_ref(sbarge, None, current_script_dir)

        # Control system
        barge.Anti_Grounding_MPC_controller(psi_d=psi_d0, MPC_prediction_horizon=5, MPC_time_step=t_step, min_depth=3.15, max_depth=3.3, sim_scale=sim_scale, weights=weights)
        #barge.u.append(PID_controller(barge, psi_d0, t_step))
        
        u_mpc = barge.get_next_ag_step()
        barge.Np.append(u_mpc[0][0])
        barge.u.append(u_mpc[1][0])

        # Navigation system
        t10, y0_zeta = barge.differential_algebraic_model(t_step)
        barge.dynamic_ontology_update(copy.deepcopy(y0_zeta), copy.deepcopy(psi_d0), copy.deepcopy(t10), copy.deepcopy(time))
        
        #t11, y1_zeta = sbarge.differential_algebraic_model(t_step)
        #sbarge.dynamic_ontology_update(copy.deepcopy(y1_zeta), copy.deepcopy(psi_d1), copy.deepcopy(t11), copy.deepcopy(time))

        # Fixed: Evaluate using the latest state instead of looping over the entire historical list `yx`
        if barge.ontology.yx:
            try:
                last_yx = barge.ontology.yx[-1]
                last_yy = barge.ontology.yy[-1]
                route_index = (np.array(x_ref_route) > last_yx).tolist().index(True)
                route_index = max(0, route_index - 1)
                x_route_index = x_ref_route[route_index]
                y_route_index = y_ref_route[route_index]
                
                psi_route_index = 0
                if route_index > 0 and route_index + 1 < len(x_ref_route):
                    psi_route_index = np.atan2(y_ref_route[route_index+1] - y_route_index, x_ref_route[route_index+1] - x_route_index)
            except (ValueError, IndexError):
                psi_route_index = 0
                x_route_index = x_ref_route[-1]
                y_route_index = y_ref_route[-1]

            current_depth = get_channel_depth(sim_scale * barge.ontology.x[-1], -sim_scale * barge.ontology.y[-1], 1200, 800, min_depth=3.15, max_depth=3.3)

        # Draw the simulation
        shore_top, shore_bottom = create_inland_depth_varying_map(screen, barge, font, sim_scale, min_depth=3.15, max_depth=3.3)
        draw_vessels_no_map(screen, [barge], sim_scale, d, time, times_to_draw, look_ahead)
        calc_bank_distances_shore(screen, [barge], sim_scale, times_to_draw, shore_top, shore_bottom)
        
        current_depth = get_channel_depth(sim_scale * barge.ontology.x[-1], -sim_scale * barge.ontology.y[-1], 1200, 800, min_depth=3.15, max_depth=3.3)
        if -current_depth + 0.2 + barge.ontology.Td[0] + barge.state[5] > 0:
            print(f"GROUNDING HAZARD. Negative Under Keel Clearance")
        else:
            print(f"Safe passage. Under-keel clearance: {round(current_depth - barge.ontology.Td[0] - barge.state[5], 2)}m.")

        pygame.display.update()
        video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False)
        time += t_step
        clock.tick(300)
    video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1), inverted=False)
    video.export(verbose=True)
    # with open('output/ag_data.csv','w',newline='') as f:
    #     writer=csv.writer(f)
    #     row_title=['t_vals','x_vals','y_vals','z_vals','U_vals','UKC_vals','sensor_x_vals','sensor_y_vals','sensor_psi_vals','psi_vals','psi_d_vals','u_mpc_delta_vals','u_mpc_np_vals']
    #     writer.writerow(row_title)
    #     for i in range(len(barge.ontology.time)):
    #         data=[barge.ontology.time[i],barge.ontology.x[i],barge.ontology.y[i],barge.ontology.zhat[i],barge.ontology.SOG[i],barge.ontology.UKC[i],barge.ontology.yx[i],barge.ontology.yy[i],barge.ontology.ypsi[i],barge.ontology.psi[i],barge.ontology.psi_d[i],barge.ontology.rudder_angle[i],barge.ontology.propeller_speed[i]]
    #         writer.writerow(data)
    plots_grounding([barge],shore_top,shore_bottom,suppress_output=False)
    pygame.display.update()
    #sbarge.reset()
    barge.reset()
    pygame.quit()