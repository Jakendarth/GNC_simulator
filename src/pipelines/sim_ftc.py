import pygame
from classes import vessel
import numpy as np
from plots import *
from traffic_planner import *
from intel_guidance import plan_route, compute_LOS_ref_adaptive
import vidmaker
import csv
import json

def run_simulation(screen,ontology,file_path,font):
    """
    Simulates the navigation of an inland vessel and visualizes it using Pygame.
    Parameters:
        screen (pygame.Surface): The Pygame surface to draw the simulation on.
        myvessel (Vessel): An instance of the Vessel class representing the vessel being simulated.
        sim_time (float): The total simulation time.
        t_step (float): The time step for each iteration of the simulation.
    Returns:
        None
    """
    #=========================================
    # Initialize Pygame parameters
    #=========================================
    pygame.display.set_caption('ASKO Vessel Case study')
    time=0
    clock = pygame.time.Clock()
    running = True
    with open(file_path) as f:
        d = json.load(f)
        sim_time = d['Simulation Time']
        t_step = d['Time step']
        if d['Own vessel name'] == 'Tito-Neri':
            own_vessel_type = 't'
        elif d['Own vessel name'] == 'Pusher-Barge':
            own_vessel_type = 'b'
        elif d['Own vessel name'] == 'ASKO':
            own_vessel_type = 'a'
        else:
            print('Vessel type not yet in database! Please try another vessel')
        in_cond_own = d['Initial conditions [own]']['Velocity'] + d['Initial conditions [own]']['Yaw rate'] + d['Initial conditions [own]']['Position'] + d['Initial conditions [own]']['Heading']
        start_point_own=d['Initial conditions [own]']['Position'][0:2]
        end_point_own=d['End point [own]'][0:2]
        fault_condition_own=d['Sensor faults']['Own']
        time_faults_own=d['Sensor faults']['Time own']
        sim_scale=d['scale']
        path_def=d['Path definition']
        look_ahead = d['Lookahead distance']
    myvessel = vessel(own_vessel_type, ontology, in_cond_own)
    myvessel.add_sensor_faults(fault_condition_own,time_faults_own)
    myvessel.noise=0.03
    myvessel.ontology.force_switch = [1, 1, 1, 0]
    myvessel.route=plan_route(start_point_own,end_point_own,path_def,2)
    # Initialize ontology lists with initial state
    myvessel.sensor_output()  # Initialize sensor output
    #=========================================
    # Initialize lists to store simulation data
    #=========================================
    us_vals=[]
    video = vidmaker.Video(path="output/case_study_frame.mp4",fps=30,late_export=True)
    #=========================================
    # Run the simulation
    #=========================================
    ep=0
    d=0  # Initialize d for drawing circles
    image = pygame.transform.scale(pygame.image.load('assets/case_study_ASKO.png').convert(),(1200, 800))
    # Draw initial frame before loop
    screen.fill((173, 216, 230))
    if image is not None:
        screen.blit(image, (0, 0))
    screen.blit(font.render('Initializing simulation...', True, "black"), (0, 0))
    pygame.display.update()
    
    while running and (time < sim_time) and (myvessel.state[3]<myvessel.route[0][-1]):
        for event in pygame.event.get():
            if (event.type == pygame.QUIT):
                running = False
        # Pass simulation dt to avoid ESO integration over wall-clock sized intervals
        myvessel.sim_dt = t_step
        psi_d,ep=compute_LOS_ref_adaptive(myvessel, ep, look_ahead, None)
        myvessel.ontology.force_switch = [1, 1, 1, 0]
        print('Here_1')
        myvessel.u.append(PID_controller(myvessel, psi_d, t_step))                        # Compute the control input
        print(myvessel.u[-1])
        t1,y_zeta=myvessel.differential_algebraic_model(t_step)                            # Compute the state update
        x_zeta=y_zeta[1]
        y_zeta=y_zeta[0] 
        myvessel.sensor_output()
        myvessel.time=time 
        print('Here_1')
        #=========================================
        # Append simulation data to lists
        #=========================================
        myvessel.ontology.x.extend(x_zeta[3].tolist())
        myvessel.ontology.y.extend(x_zeta[4].tolist())
        us_vals.extend(y_zeta[0].tolist())
        myvessel.ontology.psi.extend(x_zeta[-1].tolist())
        usq=[i**2 for i in x_zeta[0].tolist()]
        vsq=[i**2 for i in x_zeta[1].tolist()]
        Usq=[sum(x) for x in zip(usq, vsq)]
        myvessel.ontology.SOG.extend([np.sqrt(i) for i in Usq])
        myvessel.ontology.fhat.extend(myvessel.fhatu)
        myvessel.ontology.xhat.extend(myvessel.uhat)
        myvessel.ontology.Omega.extend(myvessel.Omega)
        myvessel.ontology.time.extend([tt+time for tt in t1])
        #=========================================
        # Draw the simulation
        #=========================================
        screen.fill((173, 216, 230))  # Light blue water background
        #-----------------------------------------
        # Draw canal bounds
        #-----------------------------------------
        if image is not None:
            screen.blit(image, (0, 0))
        
        # Draw text overlays with safety checks
        if len(myvessel.ontology.time) > 0:
            screen.blit(font.render('Time: '+str(round(myvessel.ontology.time[-1],2)), True, "black"), (0, 0))
        if len(myvessel.u) > 0:
            screen.blit(font.render('Rudder Command: '+str(round(myvessel.u[-1],2)), True, "black"), (0, 600))
        screen.blit(font.render('Error: '+str(round(myvessel.error,2)), True, "black"), (0,700))
        screen.blit(font.render('Reference: '+str(round(myvessel.chi[-1],2)), True, "black"), (0,650))
        print('Here_2')
        #-----------------------------------------
        # Draw the vessels
        #-----------------------------------------
        time=time+t_step
        ship_x = sim_scale*myvessel.ontology.x[-1] 
        ship_y = -sim_scale*myvessel.ontology.y[-1] 
        draw_ellipse_angle(screen, "blue", [ship_x-sim_scale*myvessel.ontology.L[0]/2, ship_y-sim_scale*myvessel.ontology.B[0]/2, sim_scale*myvessel.ontology.L[0], sim_scale*myvessel.ontology.B[0]], myvessel.ontology.psi[-1]*180/np.pi)
        pygame.draw.circle(screen, "orange", (ship_x, ship_y), sim_scale*d*myvessel.ontology.L[0]/2, 1)
        pygame.draw.circle(screen, "red", (ship_x, ship_y), sim_scale*myvessel.ontology.L[0]/2,1)
        pygame.draw.circle(screen, "cyan", (ship_x, ship_y), sim_scale*3*d*myvessel.ontology.L[0]/2, 1)
        print('Here_3')
        #-----------------------------------------
        # Draw the route
        #-----------------------------------------
        for k in range(0,len(myvessel.route[0])-1):
            pygame.draw.line(screen, "gray", (sim_scale*myvessel.route[0][k] ,-sim_scale*myvessel.route[1][k]), (sim_scale*myvessel.route[0][k+1],-sim_scale*myvessel.route[1][k+1]), 2)
        for k in range(0,len(myvessel.route[0])-1):
            if (myvessel.ontology.x[-1]>=myvessel.route[0][k] and myvessel.ontology.x[-1]<=myvessel.route[0][k+1]):
                pygame.draw.line(screen, "red", (sim_scale*myvessel.route[0][k] ,-sim_scale*myvessel.route[1][k]), (sim_scale*myvessel.route[0][k+1],-sim_scale*myvessel.route[1][k+1]), 2)
        #-----------------------------------------
        # Draw the distance from the bank
        #-----------------------------------------
        myvessel.distance_from_bank1=0.001
        myvessel.distance_from_bank2=0.001
        t_len = len(t1.tolist())
        myvessel.ontology.yx.extend(y_zeta[3].tolist())
        myvessel.ontology.yy.extend(y_zeta[4].tolist())
        myvessel.ontology.ypsi.extend(y_zeta[-1])
        yusq=[i**2 for i in y_zeta[0].tolist()]
        yvsq=[i**2 for i in y_zeta[1].tolist()]
        yUsq=[sum(x) for x in zip(yusq, yvsq)]
        myvessel.ontology.SOG.extend([np.sqrt(i) for i in yUsq])
        myvessel.ontology.zhat.extend([myvessel.z]*len(t1.tolist()))
        myvessel.ontology.psi_d.extend([float(psi_d)] * t_len)
        print('Here 4')
        #-----------------------------------------
        # Update the display
        #-----------------------------------------
        pygame.display.update()
        video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1),inverted=False)
        clock.tick(300)  # Slower frame rate for smoother motion
    myvessel.reset()
    pygame.display.update()
    #=========================================
    # Plot the simulation data
    #=========================================
    video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1),inverted=False)
    video.export(verbose=True) # save the recording (per frame)
    print("Mean beta",np.mean(((np.array([a_i - b_i for a_i, b_i in zip(myvessel.betahat, myvessel.beta)])))))
    print("Mean value cross",np.mean(myvessel.e))
    print("RMSE f1", np.sqrt(((np.array([a_i - b_i for a_i, b_i in zip(myvessel.fhatx, [0 if myvessel.tf[3]>myvessel.timehat[ii] else myvessel.f[3] for ii in range(len(myvessel.timehat))])])) ** 2).mean()))
    print("Mean f3", np.mean(((np.array([a_i - b_i for a_i, b_i in zip(myvessel.fhatpsi, [0 if myvessel.tf[-1]>myvessel.timehat[ii] else myvessel.f[-1] for ii in range(len(myvessel.timehat))])])))))
    print("Standard deviation f1", np.std(np.array([a_i - b_i for a_i, b_i in zip(myvessel.fhatx, [0 if myvessel.tf[3]>myvessel.timehat[ii] else myvessel.f[3] for ii in range(len(myvessel.timehat))])])))
    print("Standard deviation f3", np.std(np.array([a_i - b_i for a_i, b_i in zip(myvessel.fhatpsi, [0 if myvessel.tf[-1]>myvessel.timehat[ii] else myvessel.f[-1] for ii in range(len(myvessel.timehat))])])))
    print("Standard deviation beta", np.std(np.array([a_i - b_i for a_i, b_i in zip(myvessel.betahat, myvessel.beta)])))
    print("Standard deviation cross", np.std(myvessel.e))