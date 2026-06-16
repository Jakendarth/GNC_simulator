from classes import vessel
import numpy as np
from plots import *
import json
from traffic_planner import *
from intel_guidance import Guidance_Reasoner
import vidmaker
import csv

def run_simulation(screen,ontology,file_path,font):
    #=========================================
    # Initialize Pygame parameters
    #=========================================
    time=0
    clock = pygame.time.Clock()
    pygame.display.set_caption('Anti-grounding Control')
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
    myvessel = vessel(own_vessel_type, ontology, in_cond_own)
    myvessel.ontology.force_switch = [1, 1, 1, 1]
    myvessel.route=plan_route(start_point_own,end_point_own)
    myvessel.sensor_output()   
    running = True
    #=========================================
    # Initialize lists to store simulation data
    #=========================================
    myvessel.depths_data=get_depths_from_file(filename='assets/ChannDepths.csv')    # preparation: get the depth profile from file
    video = vidmaker.Video(path="output/case_study_frame.mp4",fps=30,late_export=True)
    myvessel.sensor_output()
    #=========================================
    # Run the simulation
    #=========================================
    while running and (time < sim_time) and (myvessel.state[3]<myvessel.route[0][-1]):
        for event in pygame.event.get():
            if (event.type == pygame.QUIT):
                running = False
        Dref_route_own=[0]*20
        U=2.572 # 5 knots
        d=0
        psi_d1=compute_LOS_ref(myvessel,Dref_route_own)
        myvessel.usafe=U
        myvessel.chi.append(psi_d1)
        myvessel.error=myvessel.state[-1]-psi_d1 # current yaw angle minus reference psi
        myvessel.MPC_controller(psi_d1)
        u_mpc = get_next_mpc_step(myvessel)
        myvessel.Np.append(u_mpc[0][0])
        myvessel.u.append(np.rad2deg(u_mpc[1][0])) # convert to degree
        t1,y_zeta=myvessel.differential_algebraic_model(t_step)            # Compute the state update 
        #=========================================
        # Append simulation data to lists
        #=========================================
        myvessel.ontology.x.extend([float(x) for x in y_zeta[3].tolist()])
        myvessel.ontology.y.extend([float(y) for y in y_zeta[4].tolist()])
        t1_len = len(t1.tolist())
        myvessel.ontology.yx.extend([float(myvessel.y[3])] * t1_len)
        myvessel.ontology.yy.extend([float(myvessel.y[4])] * t1_len)
        myvessel.ontology.ypsi.extend([float(myvessel.y[-1])] * t1_len)
        myvessel.ontology.psi.extend([float(p) for p in y_zeta[-1].tolist()])
        myvessel.ontology.psi_d.extend([float(psi_d1)] * t1_len)
        usq = [float(i)**2 for i in y_zeta[0].tolist()]
        vsq = [float(i)**2 for i in y_zeta[1].tolist()]
        Usq = [sum(x) for x in zip(usq, vsq)]
        myvessel.ontology.SOG.extend([float(np.sqrt(i)) for i in Usq])
        myvessel.ontology.zhat.extend([float(myvessel.z)] * t1_len)
        UKC_val = float(-myvessel.ontology.Td[0] - myvessel.z + cal_depth([myvessel.state[3], myvessel.state[4]], myvessel.depths_data))
        myvessel.ontology.UKC.extend([UKC_val] * t1_len)
        myvessel.ontology.rudder_angle.extend([float(u_mpc[1][0])] * t1_len)
        myvessel.ontology.propeller_speed.extend([float(u_mpc[0][0])] * t1_len)
        myvessel.ontology.starboard_distance.extend([float(myvessel.distance_from_bank1)] * t1_len)
        myvessel.ontology.port_distance.extend([float(myvessel.distance_from_bank2)] * t1_len)
        myvessel.ontology.time.extend([float(tt + time) for tt in t1])
        #=========================================
        # Draw the simulation
        #=========================================
        screen.fill((173, 216, 230))  # Light blue water background
        #-----------------------------------------
        # Draw canal bounds
        #-----------------------------------------
        upperbarrierRect = pygame.Rect(0, 0, 1200, 200)
        lowerbarrierRect = pygame.Rect(0, 600, 1200, 200)
        pygame.draw.rect(screen, "black", upperbarrierRect)
        pygame.draw.rect(screen, "black", lowerbarrierRect)
        pygame.draw.line(screen, "white", (0, 400), (1200, 400), 2)
        screen.blit(font.render('Port Bank', True, "white"), (550, 100))
        screen.blit(font.render('Starboard Bank', True, "white"), (550, 700))
        screen.blit(font.render('Rudder Comm: '+f"{myvessel.u[-1]:+0.2f}", True, "white"), (0, 600))
        screen.blit(font.render('Propeller n: '+str(round(myvessel.Np[-1]*60,2)), True, "white"), (300, 600))
        screen.blit(font.render('Sinkage: '+f"{myvessel.z:+.3f}", True, "white"), (300, 650))    
        screen.blit(font.render('Y: '+f"{myvessel.state[4]:+2.2f}", True, "white"), (600, 600))            
        screen.blit(font.render('UKC: '+f"{(-myvessel.ontology.Td[0]-myvessel.z+cal_depth([myvessel.state[3],myvessel.state[4]],myvessel.depths_data)):+.2f}", True, "white"), (600, 650))            
        screen.blit(font.render('Time: '+str(round(myvessel.ontology.time[-1],2)), True, "white"), (0, 0))
        screen.blit(font.render('Error: '+f"{myvessel.error:+.3f}"+'  '+f"{(np.rad2deg(myvessel.error)):+.3f}"+' (degree)', True, "white"), (0,700))
        screen.blit(font.render('Reference: '+f"{myvessel.chi[-1]:+.5f}", True, "white"), (0,650))
        #-----------------------------------------
        # Draw the vessels
        #-----------------------------------------
        ship_x = myvessel.ontology.x[-1]+200
        ship_y = -myvessel.ontology.y[-1]+400
        draw_ellipse_angle(screen, "blue", [ship_x-myvessel.ontology.L[0]/2, ship_y-myvessel.ontology.B[0]/2, myvessel.ontology.L[0], myvessel.ontology.B[0]], myvessel.ontology.psi[-1]*180/np.pi)
        pygame.draw.circle(screen, "orange", (ship_x, ship_y), 2*d*myvessel.ontology.L[0]/2, 1)
        pygame.draw.circle(screen, "red", (ship_x, ship_y), myvessel.ontology.L[0]/2,1)
        #-----------------------------------------
        # Draw the route
        #-----------------------------------------
        for k in range(0,len(myvessel.route[0])-1):
            pygame.draw.line(screen, "gray", (myvessel.route[0][k]+200,-myvessel.route[1][k]+400), (myvessel.route[0][k+1]+200,-myvessel.route[1][k+1]+400), 2)
        for k in range(0,len(myvessel.route[0])-1):
            if (myvessel.ontology.x[-1]>=myvessel.route[0][k] and myvessel.ontology.x[-1]<=myvessel.route[0][k+1]):
                pygame.draw.line(screen, "red", (myvessel.route[0][k]+200,-myvessel.route[1][k]+400), (myvessel.route[0][k+1]+200,-myvessel.route[1][k+1]+400), 2)
        #-----------------------------------------
        # Draw the distance from the bank
        #-----------------------------------------
        pygame.draw.line(screen, "white", (ship_x-myvessel.ontology.B[0]/2*np.sin(myvessel.ontology.psi[-1]), ship_y-myvessel.ontology.B[0]/2*np.cos(myvessel.ontology.psi[-1])), (ship_x, 200), 2)
        pygame.draw.line(screen, "white", (ship_x+myvessel.ontology.B[0]/2*np.sin(myvessel.ontology.psi[-1]), ship_y+myvessel.ontology.B[0]/2*np.cos(myvessel.ontology.psi[-1])), (ship_x, 600), 2)
        myvessel.distance_from_bank1=np.sqrt((myvessel.ontology.B[0]/2*np.sin(myvessel.ontology.psi[-1]))**2+(ship_y-np.sign(myvessel.distance_from_bank1-myvessel.distance_from_bank2)*myvessel.ontology.B[0]/2*np.cos(myvessel.ontology.psi[-1])-200)**2)
        myvessel.distance_from_bank2=np.sqrt((myvessel.ontology.B[0]/2*np.sin(myvessel.ontology.psi[-1]))**2+(ship_y+np.sign(myvessel.distance_from_bank1-myvessel.distance_from_bank2)*myvessel.ontology.B[0]/2*np.cos(myvessel.ontology.psi[-1])-600)**2)
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
    for l in [myvessel.ontology.time,myvessel.ontology.x,myvessel.ontology.y,myvessel.ontology.zhat,myvessel.ontology.SOG,myvessel.ontology.UKC,myvessel.ontology.yx,myvessel.ontology.yy,myvessel.ontology.ypsi,myvessel.ontology.psi,myvessel.ontology.psi_d,myvessel.ontology.rudder_angle,myvessel.ontology.propeller_speed]:
        l.remove(0)
    video.update(pygame.surfarray.pixels3d(screen).swapaxes(0, 1),inverted=False)
    video.export(verbose=True) # save the recording (per frame)
    with open('output/case_study_route.csv','w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['x_ref','y_ref'])
        x_ref_route=myvessel.route[0]
        y_ref_route=myvessel.route[1]
        for i in range(len(x_ref_route)):
            writer.writerow([x_ref_route[i],y_ref_route[i]])
    with open('output/case_study_data.csv','w',newline='') as f:
        writer=csv.writer(f)
        row_title=['t_vals','x_vals','y_vals','z_vals','U_vals','UKC_vals','sensor_x_vals','sensor_y_vals','sensor_psi_vals','psi_vals','psi_d_vals','u_mpc_delta_vals','u_mpc_np_vals']
        writer.writerow(row_title)
        for i in range(len(myvessel.ontology.time)):
            data=[myvessel.ontology.time[i],myvessel.ontology.x[i],myvessel.ontology.y[i],myvessel.ontology.zhat[i],myvessel.ontology.SOG[i],myvessel.ontology.UKC[i],myvessel.ontology.yx[i],myvessel.ontology.yy[i],myvessel.ontology.ypsi[i],myvessel.ontology.psi[i],myvessel.ontology.psi_d[i],myvessel.ontology.rudder_angle[i],myvessel.ontology.propeller_speed[i]]
            writer.writerow(data)
    plots_grounding(myvessel,suppress_output=True)
    myvessel.reset()
    pygame.display.update()    