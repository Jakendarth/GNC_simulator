import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
import cvxpy as cp
from scipy.integrate import odeint, solve_ivp
import os
   
def Finite_State_Machine(gparams,dsafe,myvessel,time,sim_scale):
    """
    Determines the role of the vessel and various state conditions based on the given parameters.
    Parameters:
        psi_c (float): Relative heading between vessels.
        psi_b (float): Relative bearing of vessels.
        d (float): Distance to the contact vessel.
        d_l (float): Lane distance.
        psi (float): Heading of the own vessel.
        dcpa (float): Distance at closest point of approach.
        rho (float): Safety distance.
        rho_i (float): Additional safety distance.
        rho_s (float): Safety distance for static obstacles.
        rho_enc (float): Encounter distance threshold.
        rho_emg (float): Emergency distance threshold.
        myvessel (object): The vessel object which contains ontology attributes.
    Returns:
        None: The function updates the role of the vessel and sets various state conditions.
    """
    # gparams--->(psi_c, psi_b, d, d_cpa)
    psi_h=6*np.pi/180
    psi_c=gparams[0]%(2*np.pi)
    psi_b=gparams[1]%(2*np.pi)
    w=1600 
    d_lc0=0.5*w
    if (sim_scale*myvessel.y[3]>=600):
        myvessel.ratio_head_on.append(d_lc0/myvessel.distance_from_bank1-(myvessel.ontology.L[0]*np.tan(psi_c))/myvessel.distance_from_bank1-1)
        myvessel.head_on_upperbound.append((0.8*w/myvessel.distance_from_bank1)-1)
        myvessel.head_on_lowerbound.append((0.5*myvessel.ontology.B[0])/(myvessel.distance_from_bank1))
        myvessel.ratio_time.append(time)
    Tenc=gparams[2]<=5*myvessel.ontology.L[0]
    Trsk=gparams[3]<=dsafe+myvessel.ontology.L[0]
    Thdn=(psi_c>=np.pi-psi_h) and (psi_c<=np.pi+psi_h) 
    Tstr=(psi_c>=3*np.pi/8) and (psi_c<(np.pi-psi_h))#(psi_c>=np.pi+psi_h) and (psi_c<13*np.pi/8)
    Tbrn=(psi_c>=13*np.pi/8) and (psi_c<3*np.pi/8)
    Tovr=(np.pi+psi_b-psi_c>=5*np.pi/8) and (np.pi+psi_b-psi_c<11*np.pi/8)
    Tstb=(psi_b>=0) and (psi_b<5*np.pi/8)
    Temg=gparams[2]<myvessel.ontology.L[0]
    Tent_GW=Tenc and (Trsk and (Thdn or Tstr or (Tbrn and (Tovr or Tstb))))
    Tbpr=np.abs(myvessel.distance_from_bank1)>=np.abs(myvessel.distance_from_bank2)
    Tent_GW_inland= Tenc and (Trsk and Tbpr and(((Thdn or Tstr) or (not Tstb)) or (Tbrn and (Tovr or Tstb))))
    T_inland=(sim_scale*myvessel.ontology.yx[-1]>=600)
    Text_GW=not Tenc 
    Tent_EM=Temg 
    Text_EM=not Temg
    if ((psi_c>np.pi/2) and (psi_b<3*np.pi/2) and (abs(psi_b-psi_c)<np.pi/2)):
        myvessel.scenario='Safe'
    else:
        if ((psi_c>=(np.pi-psi_h)) and (psi_c<(np.pi+psi_h))):
            myvessel.scenario='Head On'
        elif ((psi_c>=(np.pi+psi_h)) and (psi_c<13*np.pi/8)):
            myvessel.scenario='Port crossing'
        elif ((psi_c>=3*np.pi/8) and (psi_c<(np.pi-psi_h))):
            myvessel.scenario='Starboard crossing'
        elif ((psi_b>=5*np.pi/8) and (psi_b<11*np.pi/8)):
            myvessel.scenario='Overtaking'
        elif ((((np.pi+psi_b-psi_c)%(2*np.pi))>=5*np.pi/8) and (((np.pi+psi_b-psi_c)%(2*np.pi))<11*np.pi/8)):
            myvessel.scenario='Overtaken'
        elif (psi_b<np.pi):
            myvessel.scenario='Starboard crossing'
        else:
            myvessel.scenario='Port crossing'
    if (myvessel.type=="Other4"):
        print("Am I in an inland waterway?", T_inland)
        print("Should I give-way?", Tent_GW_inland)
        print("Did someone enter my encounter radius?", Tenc )
        print("Is it a head-on?",Thdn)
        print("Does BPR make me a give-way?",Tbpr)
        print("Does stb not hold?", not Tstb)
    if (Tent_GW and (not T_inland)) or ((Tent_GW_inland) and T_inland) or (Tent_EM):
        myvessel.role='Give-way'
    elif (Text_GW or Text_EM):
        myvessel.role='Stand-On'
            

def Guidance_Reasoner(time, myvessel, other_vessels, ontology, sim_scale,current_script_dir):
    # ================================================
    # 1. Calculation of geometrical parameters for the encounter scenario
    #=================================================
    path_def=len(myvessel.route[0])
    enc_params=[]
    for instance1 in other_vessels:
        t_cpa, d_cpa, d = compute_CPA(myvessel, instance1)
        if (d<=5*myvessel.ontology.L[0]):
            psi_c=instance1.y[-1]-myvessel.y[-1]
            psi_b=np.arctan2(np.dot(np.array([np.sin(myvessel.y[-1]),np.cos(myvessel.y[-1])]), np.array([instance1.y[3],instance1.y[4]])-np.array([myvessel.y[3],myvessel.y[4]])),np.dot(np.array([np.cos(myvessel.y[-1]),-np.sin(myvessel.y[-1])]), np.array([instance1.y[3],instance1.y[4]])-np.array([myvessel.y[3],myvessel.y[4]])))
            enc_params.append((psi_c, psi_b, d, d_cpa))
    # ================================================
    # 2. Configure applicable regulations
    #=================================================
    # Access individuals in the class 'Regulations'
    regulations_class = ontology.search_one(iri="*Regulations")
    active_regs=[]
    tobechecked_regs=[]
    Usafe=2.5
    dsafe=myvessel.ontology.B[0]/2
    reg_set=""
    for individual in list(regulations_class.instances()):
        if sim_scale*myvessel.y[3]<=600:
            reg_set="COLREGS"
            if "COLREGS/Situation_Invariant" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(1)
                    active_regs.append(subindividual)
            elif "COLREGS/Situation_Analysis" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(1)
                    active_regs.append(subindividual) 
            elif "COLREGS/Situation_Dependent" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0)
                    tobechecked_regs.append(subindividual)
            elif "BPR/Situation_Invariant" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0)
            elif "BPR/Situation_Analysis" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0)
            elif "BPR/Situation_Dependent" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0) 
        else:
            reg_set="BPR"
            if "BPR/Situation_Invariant" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(1)
                    active_regs.append(subindividual)
            elif "BPR/Situation_Analysis" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(1)
                    active_regs.append(subindividual)
            elif "BPR/Situation_Dependent" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    tobechecked_regs.append(subindividual) 
                    subindividual.reg_active.append(0) 
            elif "COLREGS/Situation_Invariant" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0)
            elif "COLREGS/Situation_Analysis" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0)
            elif "COLREGS/Situation_Dependent" in individual.name:
                for subindividual in individual.INDIRECT_equivalent_to:
                    subindividual.reg_active.append(0)
    # ================================================
    # 3. Scenario classification  and role assignment [FSM]
    #=================================================
    vessel_index_in_radius=0
    if (len(enc_params)>=1): # there are target ships to avoid
        for ii in range(0,len(enc_params)): 
            Finite_State_Machine(enc_params[ii], dsafe, myvessel, time, sim_scale)
            vessel_index_in_radius=ii
    else:
        myvessel.scenario="Safe"
        myvessel.role="Stand-On"
    # ================================================
    # 4. Calculation of own path deviation
    #================================================="""
    Dx1=np.zeros(len(myvessel.route[0])).tolist()
    Dy1=np.zeros(len(myvessel.route[0])).tolist()
    Droute_ref_own=(np.zeros(len(myvessel.route[0])).tolist(),np.zeros(len(myvessel.route[1])).tolist())
    # Calculate distance between vessel and boundaries d_{WEj} and check against safe distance 
    #print(myvessel.role)
    if (myvessel.role=="Stand-On"):
        pass
    else:
        if (reg_set=="BPR"):
            route_points=[np.array([myvessel.route[0][ii], myvessel.route[1][ii]]) for ii in range(0,len(myvessel.route[0]))]
            distance_from_current=[np.sqrt((route_points[jj][0] - myvessel.ontology.yx[-1])**2 + (route_points[jj][1] - myvessel.ontology.yy[-1])**2 ) for jj in range(0,len(route_points))]
            pos=np.argmin(distance_from_current)
            print(myvessel.type, ' has route ', route_points)
            print(myvessel.type, ' has position ', [myvessel.y[3], myvessel.y[4]])
            print("Vessel just passed waypoint", pos)
            Dx1, Dy1= generate_path_envelope(myvessel, other_vessels[vessel_index_in_radius],path_def,dsafe,1,pos,Dx1,Dy1,current_script_dir)
            Droute_ref_own=(np.asarray(Dx1),np.asarray(Dy1))
        else:
            route_points=[np.array([myvessel.route[0][ii], myvessel.route[1][ii]]) for ii in range(0,len(myvessel.route[0]))]
            distance_from_current=[np.sqrt((route_points[jj][0] - myvessel.y[3])**2 + (route_points[jj][1] - myvessel.y[4])**2 ) for jj in range(0,len(route_points))]
            two_lowest=np.partition(distance_from_current, 1)[0:2]
            sum_two_lowest=np.sum(two_lowest)
            print("Two lowest (sum)",sum_two_lowest)
            pos=np.argmin(distance_from_current)
            #print(myvessel.type, ' has route ', route_points)
            #print(myvessel.type, ' has position ', [myvessel.y[3], myvessel.y[4]])
            #print("Vessel just passed waypoint", pos)
            Dx1, Dy1= generate_path_envelope(myvessel, other_vessels[vessel_index_in_radius],path_def,dsafe,0,pos,Dx1,Dy1,current_script_dir)
            Droute_ref_own=(np.asarray(Dx1),np.asarray(Dy1))
    myvessel.droute=Droute_ref_own
    myvessel.Dx=Dx1
    myvessel.Dy=Dy1                     
    return Droute_ref_own, Usafe, dsafe, myvessel.scenario,other_vessels[vessel_index_in_radius]


def generate_path_envelope(myvessel, other_vessel,path_def,dsafe,where,pos,Dx_old,Dy_old,current_script_dir):
    Dx=np.zeros(len(myvessel.route[0])).tolist()
    Dy=np.zeros(len(myvessel.route[1])).tolist()
    with open(os.path.join(current_script_dir,"input/SimParam_WC.json")) as f:
        d = json.load(f)
        look_ahead=d['Lookahead distance']
    rho_s=myvessel.ontology.B[0]/2#myvessel.ontology.L[0]*dsafe
    rho_p= myvessel.ontology.L[0]/2
    rho_q= other_vessel.ontology.L[0]/2
    w=np.abs(myvessel.distance_from_bank1)+np.abs(myvessel.distance_from_bank2)#4*myvessel.ontology.L[0]
    yref= myvessel.route[1]
    xref= myvessel.route[0]
    yo=other_vessel.ontology.yy[-1]
    xo=other_vessel.ontology.yx[-1]
    psi_o=other_vessel.ontology.ypsi[-1]
    k=pos
    if (where==0):
        while ((k<path_def-1)):
            # Compute auxiliary terms
            phi_1=(rho_s+rho_p+rho_q)*np.sqrt(1+np.tan(psi_o)**2)+ np.abs((yo-yref[k+1])+(xref[k+1]-xo)*np.tan(psi_o))
            a = cp.Variable()
            b = cp.Variable()
            c= cp.Variable()
            constraints1 = [ 
                np.sign(-np.cos(myvessel.ontology.ypsi[-1]))*(b-a*np.tan(psi_o))>= phi_1,
                #myvessel.ontology.L[0]/2*b+a*(-myvessel.ontology.L[0]/2)-myvessel.ontology.L[0]/2*(-myvessel.ontology.L[0]/2)<=0,
                b*np.sign(-np.cos(myvessel.ontology.ypsi[-1]))>=0,
                #a*np.sign(np.cos(myvessel.y[-1]))>=0,
                #myvessel.ontology.L[0]/2*a+b*(-myvessel.ontology.L[0]/2)-myvessel.ontology.L[0]/2*(-myvessel.ontology.L[0]/2)<=0,
                #a<=myvessel.ontology.L[0]/2,
                #a>=-myvessel.ontology.L[0]/2,
                #b<=myvessel.ontology.L[0]/2,
                #b>=myvessel.ontology.L[0]/2,
                #b*np.sign(-np.cos(myvessel.y[-1]))>=0,
                #a*np.sign(-np.cos(myvessel.y[-1]))<=0,
                #np.sign(-np.cos(myvessel.y[-1]))*b>=0,
            ]
            problem = cp.Problem(cp.Minimize(a**2+b**2), constraints1)
                

            # Solve the problem
            problem.solve()
            # Extract the results
            delta_x_ref = a.value
            delta_y_ref = b.value
            if (delta_x_ref is not None) and (delta_y_ref is not None):
                Dx[k+1]= delta_x_ref
                Dy[k+1]= delta_y_ref
            else:
                Dx[k+1]=Dx_old[k+1]
                Dy[k+1]=Dy_old[k+1]
            k+=1
    else:
        while (k<path_def-1):
            # Compute auxiliary terms
            phi_1=(rho_s+rho_p+rho_q)*np.sqrt(1+np.tan(psi_o)**2)+ np.abs((-yo+yref[k+1])+(-xref[k+1]+xo)*np.tan(psi_o))
            phi_2=0.8*w+ np.abs(yref[k+1]-200)
            phi_3=0.8*w+ np.abs(yref[k+1]+w/2)
            # Compute bounds for path envelope
            # Define variables
            a = cp.Variable()
            b = cp.Variable()

            # Define constraints
            constraints1 = [
                np.sign(-np.cos(myvessel.y[-1]))*(b-a*np.tan(psi_o)) >= phi_1,
                np.sign(-np.cos(myvessel.y[-1]))*b>=0,
                #b*myvessel.y[0]<=0,
                b <= phi_2,
            ]
            # Define the optimization problem
            problem = cp.Problem(cp.Minimize(a**2+b**2), constraints1)

            # Solve the problem
            problem.solve()

            # Extract the results
            delta_x_ref = a.value
            delta_y_ref = b.value
            if (delta_x_ref is not None) and (delta_y_ref is not None):
                Dx[k+1]= delta_x_ref
                Dy[k+1]= delta_y_ref
            else:
                #print("Here")
                Dx[k+1]= Dx_old[k+1]
                Dy[k+1]= Dy_old[k+1]
            k+=1
    return Dx, Dy
    

def compute_CPA(myvessel, instance):
    d=np.sqrt((myvessel.ontology.yx[-1]-instance.ontology.yx[-1])**2+(myvessel.ontology.yy[-1]-instance.ontology.yy[-1])**2)
    Rp=np.array([[np.cos(myvessel.ontology.ypsi[-1]),-np.sin(myvessel.ontology.ypsi[-1])],[np.sin(myvessel.ontology.ypsi[-1]),np.cos(myvessel.ontology.ypsi[-1])]])
    Rq=np.array([[np.cos(instance.ontology.ypsi[-1]),-np.sin(instance.ontology.ypsi[-1])],[np.sin(instance.ontology.ypsi[-1]),np.cos(instance.ontology.ypsi[-1])]])
    vp=np.array([myvessel.state[0],myvessel.state[1]])
    vq=np.array([instance.state[0],instance.state[1]])
    term11=Rp.dot(vp)-Rq.dot(vq)  
    term21=-np.array([instance.y[3],instance.y[4]])+np.array([myvessel.y[3],myvessel.y[4]])  # p-q
    t_cpa=-(np.transpose(term11).dot(term21))/(np.linalg.norm(term11))**2
    d_cpa=d
    if t_cpa>=0:
        d_cpa=np.linalg.norm(term21+term11*t_cpa)
    else:
        d_cpa=d
    return t_cpa, d_cpa, d

def compute_CPA_env(myvessel,other_vessel,tcpa,dsafe):
    t_min=0
    t_max=np.inf
    if (tcpa>=0):
        d=np.sqrt((myvessel.y[3]-other_vessel.y[3])**2+(myvessel.y[4]-other_vessel.y[4])**2)
        Rp=np.array([[np.cos(myvessel.y[-1]),-np.sin(myvessel.y[-1])],[np.sin(myvessel.y[-1]),np.cos(myvessel.y[-1])]])
        Rq=np.array([[np.cos(other_vessel.y[-1]),-np.sin(other_vessel.y[-1])],[np.sin(other_vessel.y[-1]),np.cos(other_vessel.y[-1])]])
        vp=np.array([myvessel.y[0],myvessel.y[1]])
        vq=np.array([other_vessel.y[0],other_vessel.y[1]])
        term11=Rp.dot(vp)-Rq.dot(vq)  
        term21=-np.array([other_vessel.y[3],other_vessel.y[4]])+np.array([myvessel.y[3],myvessel.y[4]])  # p-q
        t_min=(-dsafe*myvessel.ontology.L[0]+d)/np.linalg.norm(term11)
        if (myvessel.y[3]>1600):
            t_max= -(0.8*4*myvessel.ontology.L[0]-myvessel.distance_from_bank2-np.linalg.norm(term21))/np.linalg.norm(term11)
    return t_min, t_max

def compute_LOS_ref_adaptive(vessel,d_route,current_script_dir):
    chi_rate_max = np.pi/90
    with open(os.path.join(current_script_dir,"input/SimParam.json")) as f:
        d = json.load(f)
        look_ahead=d['Lookahead distance']
    x_los = [vessel.ontology.yx[-1], vessel.ontology.yy[-1], vessel.ontology.ypsi[-1], vessel.ontology.SOG[-1]]  # Current state (x,y,psi,U)
    K_p = 1 / look_ahead  # Proportional gain for the LOS algorithm. K_p = 1 / lookahead distance.
    wp_pos = (vessel.route[0]+d_route[0], vessel.route[1]+d_route[1])
    n_wps = wp_pos[0].shape[0]  # Number of waypoints
    for i in range(vessel.wp_idx, n_wps - 1):
        d_0wp_vec = [a_i-b_i for a_i,b_i in zip([wp_pos[0][i + 1],wp_pos[1][i + 1]],x_los[:2])]
        L_wp_segment = [a_i-b_i for a_i,b_i in zip([wp_pos[0][i + 1],wp_pos[1][i + 1]],[wp_pos[0][i],wp_pos[1][i]])]
        if (vessel.type=="Other2"):
            print("Waypoint distance vector:", L_wp_segment)
            print("My (x,y) is ",x_los[:2])
            print("My distance vector from next waypoint:", d_0wp_vec)
        segment_passed = vessel.check_for_wp_segment_switch(L_wp_segment, d_0wp_vec)
        if segment_passed:
            vessel.wp_idx += 1
        else:
            break
    mu=0.08
    Uc_max=0.5
    epsilon=0.001
    try:
        Uc=-0.000002*vessel.distance_from_bank1**2+0.002*vessel.distance_from_bank1
    except:
        Uc=0
    uc=-Uc 
    vc=0
    vessel.beta.append(np.arctan2(vessel.state[1],vessel.state[0]))# Sideslip angle
    beta_c=-np.arctan2((vc),(uc))  # Bearing of the vehicle
    if vessel.wp_idx < n_wps - 1: #and np.sqrt((vessel.y[3] - wp_pos[0][vessel.wp_idx])**2 + (vessel.y[4] - wp_pos[1][vessel.wp_idx])**2) > 10:
        L_wp_segment = [a_i-b_i for a_i,b_i in zip([wp_pos[0][vessel.wp_idx + 1],wp_pos[1][vessel.wp_idx + 1]],[wp_pos[0][vessel.wp_idx],wp_pos[1][vessel.wp_idx]])]
        alpha = np.arctan2(L_wp_segment[1], L_wp_segment[0])
        #y_e = -(x_los[0]-wp_pos[0][vessel.wp_idx]) * np.sin(alpha) + (x_los[1]-wp_pos[1][vessel.wp_idx]) * np.cos(alpha) #Cross-track error
        #vessel.e=y_e
        #print('alpha',alpha)
        vessel.alpha=alpha
        #vessel.e.append(-(vessel.y[3] - wp_pos[0][vessel.wp_idx]) * np.sin(alpha) + (vessel.y[4] - wp_pos[1][vessel.wp_idx]) * np.cos(alpha))
        y0=vessel.extended_state
        res = solve_ivp(vessel.eso_rhs, t_span=(0, vessel.timestep), y0=y0,method='RK45',rtol=1e-8,atol=1e-8)
        vessel.extended_state=[res.y[ii][-1] for ii in range(len(res.y))]
        #vessel.state=[vessel.extended_state[ii] for ii in range(0, 12)]
        vessel.uhat.append(vessel.extended_state[0])#=[res.y[jj][ii] for jj in range(0, 6) for ii in range(len(res.y[jj])) ]
        vessel.psihat.append(vessel.extended_state[5])
        vessel.Omega.append(vessel.extended_state[6])#=[res.y[jj][ii] for jj in range(6, 12) for ii in range(len(res.y[jj])) ]
        vessel.fhatu.append(vessel.extended_state[12])#=[res.y[jj][ii] for jj in range(12, 18) for ii in range(len(res.y[jj])) ]
        vessel.fhatpsi.append(vessel.extended_state[17])
        vessel.fhatx.append(vessel.extended_state[15])
        vessel.betahat.append(vessel.extended_state[18])
        vessel.ehat.append(vessel.extended_state[19])
        vessel.e.append(-(x_los[0]-wp_pos[0][vessel.wp_idx]) * np.sin(alpha) + (x_los[1]-wp_pos[1][vessel.wp_idx]) * np.cos(alpha))
        #vessel.e.append(vessel.extended_state[20])
        vessel.timehat.append(res.t[-1]+vessel.time)#.extend([tt+vessel.time for tt in res.t])
        #print("Fault estimation", vessel.fhat[-1])
        #print("Omega", vessel.Omega[-1])
        #vessel.state.insert(5,vessel.z)        
        # while the waypoint index is less than the number of waypoints - 1
        # and the distance to the waypoint is greater than 10m
        Uv=np.sqrt((vessel.extended_state[0])**2+(vessel.extended_state[1])**2)  
        #vessel.e=e
        e_dot= Uv*np.sin(vessel.extended_state[5]-alpha+beta_c) #- Uc*np.sin(-beta_c+alpha)  # y_dot - v_y
        #sum_e=(mu*e/np.sqrt(1+(mu*e)**2)*(1-np.abs((e-ep)/vessel.timestep)/epsilon))*vessel.timestep
        #print("Derivative of error is:", e_dot)
        #print("Distance from bank 1 is",vessel.distance_from_bank1)
        #print("Current speed is", Uc)
        if (np.abs(e_dot) < epsilon):
            yi_dot=mu*vessel.e[-1]*(1-np.abs(e_dot)/epsilon)
            yi= vessel.yi_prev + yi_dot * vessel.timestep  # Integral of the error
            #psi_r = -np.arctan(K_p * e+ K_i * yi) #- np.arcsin(x_los[2] / x_los[3] + 1e-4)
            psi_r= -np.arctan(K_p * vessel.ehat[-1] + vessel.betahat[-1])
            vessel.yi_prev = yi  # Update the integral of the error
        else:
            psi_r = -np.arctan(K_p * vessel.ehat[-1] + vessel.betahat[-1])
        #print("psi_r",psi_r)
        psi_d_raw = alpha + psi_r
        #print("psi_d_raw",psi_d_raw)
        psi_d_rate = (psi_d_raw - vessel.psi_d_prev)
        psi_d_rate = np.clip(psi_d_rate, -chi_rate_max, chi_rate_max)
        psi_d = vessel.psi_d_prev + psi_d_rate
        vessel.chi.append(psi_d)
        vessel.psi_d_prev=psi_d
        #print('psi_d',psi_d)
        return psi_d#wrap_angle_to_pmpi(psi_d)  # Desired course angle (in radians)
    #if the distance to the last waypoint is less than 10m, set the desired course angle to zero
    else:
        #print("I am vessel  and I am at the last waypoint", vessel.wp_idx, " with position ", vessel.y[3], vessel.y[4])
        psi_d = vessel.psi_d_prev
        y0=vessel.extended_state
        res = solve_ivp(vessel.eso_rhs, t_span=(0, vessel.timestep), y0=y0,method='RK45',rtol=1e-8,atol=1e-8)
        vessel.extended_state=[res.y[ii][-1] for ii in range(len(res.y))]
        #vessel.state=[vessel.extended_state[ii] for ii in range(0, 12)]
        vessel.uhat.append(vessel.extended_state[0])#=[res.y[jj][ii] for jj in range(0, 6) for ii in range(len(res.y[jj])) ]
        vessel.psihat.append(vessel.extended_state[5])
        vessel.Omega.append(vessel.extended_state[6])#=[res.y[jj][ii] for jj in range(6, 12) for ii in range(len(res.y[jj])) ]
        vessel.fhatu.append(vessel.extended_state[12])#=[res.y[jj][ii] for jj in range(12, 18) for ii in range(len(res.y[jj])) ]
        vessel.fhatpsi.append(vessel.extended_state[17])
        vessel.betahat.append(vessel.extended_state[18])
        vessel.fhatx.append(vessel.extended_state[15])
        vessel.ehat.append(vessel.extended_state[19])
        vessel.timehat.append(res.t[-1]+vessel.time)#.extend([tt+vessel.time for tt in res.t])
        #print("Fault estimation", vessel.fhat[-1])
        #print("Omega", vessel.Omega[-1])
        #vessel.state.insert(5,vessel.z)        
        # while the waypoint index is less than the number of waypoints - 1
        # and the distance to the waypoint is greater than 10m
        Uv=np.sqrt((vessel.extended_state[0])**2+(vessel.extended_state[1])**2)
        vessel.chi.append(psi_d)
        vessel.psi_d_prev=psi_d
        if (vessel.type=="Other2"):
            print("Vessel ", vessel.type, " resolves in ",psi_d)
            print("Waypoint index ",vessel.wp_idx)
            print("Number of waypoints ", n_wps - 1)
        #vessel.e.append(ep)
        return psi_d # Desired course angle (in radians) 

def compute_LOS_ref(self,d_route,current_script_dir):
    """
    Computes the desired course angle (chi_d) for a vehicle using a Line-of-Sight (LOS) guidance algorithm.
    Parameters:
        ref_route (list of lists): A 2D list representing the reference route. 
                                   ref_route[0] contains x-coordinates, and ref_route[1] contains y-coordinates.
        measurements (list): A list of measurements from the vehicle's sensors. 
                             measurements[0] is the velocity in the x-direction,
                             measurements[1] is the velocity in the y-direction,
                             measurements[3] is the current x-position,
                             measurements[4] is the current y-position.
        chi_d_prev (float): The previous desired course angle (in radians).
    Returns:
        float: The updated desired course angle (chi_d) in radians.
    """
    chi_rate_max = np.pi/90
    with open(os.path.join(current_script_dir,"input/SimParam.json")) as f:
        d = json.load(f)
        look_ahead=d['Lookahead distance']
    x_los = [self.ontology.yx[-1], self.ontology.yy[-1], self.ontology.ypsi[-1], self.ontology.SOG[-1]]  # Current state (x,y,psi,U)
    #print(x_los)
    K_p = 1 / look_ahead  # Proportional gain for the LOS algorithm. K_p = 1 / lookahead distance.
    K_i= 1/ (2*look_ahead)
    wp_pos = (self.route[0]+d_route[0], self.route[1]+d_route[1])
    n_wps = wp_pos[0].shape[0]  # Number of waypoints
    for i in range(self.wp_idx, n_wps - 1):
        d_0wp_vec = [a_i-b_i for a_i,b_i in zip([wp_pos[0][i + 1],wp_pos[1][i + 1]],x_los[:2])]
        L_wp_segment = [a_i-b_i for a_i,b_i in zip([wp_pos[0][i + 1],wp_pos[1][i + 1]],[wp_pos[0][i],wp_pos[1][i]])]

        segment_passed = self.check_for_wp_segment_switch(L_wp_segment, d_0wp_vec)
        if segment_passed:
            self.wp_idx += 1
        else:
            break
    # while the waypoint index is less than the number of waypoints - 1
    # and the distance to the waypoint is greater than 10m
    # try:
    #     Uc=-0.000002*self.distance_from_bank1**2+0.002*self.distance_from_bank1
    # except:
    #     Uc=0
    # uc=-Uc 
    # vc=0
    #Uv=np.sqrt((self.y[0])**2+(self.y[1])**2)  
    #beta_c=-np.arctan2((vc),(uc))  # Bearing of the vehicle
    if self.wp_idx < n_wps - 1: #and np.sqrt((self.y[3] - wp_pos[0][self.wp_idx])**2 + (self.y[4] - wp_pos[1][self.wp_idx])**2) > 10:
        L_wp_segment = [a_i-b_i for a_i,b_i in zip([wp_pos[0][self.wp_idx + 1],wp_pos[1][self.wp_idx + 1]],[wp_pos[0][self.wp_idx],wp_pos[1][self.wp_idx]])]
        alpha = np.arctan2(L_wp_segment[1], L_wp_segment[0])
        e = -(x_los[0]-wp_pos[0][self.wp_idx]) * np.sin(alpha) + (x_los[1]-wp_pos[1][self.wp_idx]) * np.cos(alpha)
        x_e =  (x_los[0]-wp_pos[0][self.wp_idx]) * np.cos(alpha) + (x_los[1]-wp_pos[1][self.wp_idx]) * np.sin(alpha) #Along-track error
        y_e = -(x_los[0]-wp_pos[0][self.wp_idx]) * np.sin(alpha) + (x_los[1]-wp_pos[1][self.wp_idx]) * np.cos(alpha) #Cross-track error
        self.e=y_e
        #e_dot= Uv*np.sin(self.y[-1]-alpha+beta_c) - Uc*np.sin(-beta_c+alpha)  # y_dot - v_y
        # if (np.abs(e_dot) < epsilon):
        #     yi_dot=mu*e*(1-np.abs(e_dot)/epsilon)
        #     yi= self.yi_prev + yi_dot * self.timestep  # Integral of the error
        #     psi_r = -np.arctan(K_p * e+ K_i * yi) #- np.arcsin(x_los[2] / x_los[3] + 1e-4)
        #     self.yi_prev = yi  # Update the integral of the error
        # else:
        #     psi_r = -np.arctan(K_p * e)
        psi_r = -np.arctan(K_p * y_e)
        psi_d_raw = alpha + psi_r
        psi_d_rate = (psi_d_raw - self.psi_d_prev)
        psi_d_rate = np.clip(psi_d_rate, -chi_rate_max, chi_rate_max)
        psi_d = self.psi_d_prev + psi_d_rate
        self.psi_d_prev=psi_d
        self.chi.append(psi_d)
        #print("I am vessel ", self.type, " and I am following a psi_d of ",wrap_angle_to_pmpi(psi_d))
        return psi_d #wrap_angle_to_pmpi(psi_d)  # Desired course angle (in radians)
    else:  #if the distance to the last waypoint is less than 10m, set the desired course angle to zero
        psi_d = self.psi_d_prev
        self.chi.append(psi_d)
        return psi_d # Desired course angle (in radians) 
    

def plan_route(start,stop,curve_degree,current_script_dir):
    """
    Plans a route between a start and stop point by generating reference points.
    Parameters:
        start (tuple): A tuple (x, y) representing the starting coordinates.
        stop (tuple): A tuple (x, y) representing the stopping coordinates.
    Returns:
        tuple: Two numpy arrays (x_ref, y_ref) representing the x and y coordinates 
               of the reference points along the route.
    """
    with open(os.path.join(current_script_dir,"input/SimParam_WC.json")) as f:
        d = json.load(f)
        spacing=d['Waypoint spacing']
    try:
        if (start[0]<=stop[0]):
            x_ref=np.arange(start[0],stop[0],spacing)
        else:
            x_ref=np.arange(start[0],stop[0],-spacing)
        #print(len(x_ref))
        y_ref=[0]*len(x_ref)
        if (curve_degree==1):
            y_ref=start[1]+((stop[1]-start[1])/(stop[0]-start[0]))*(x_ref-start[0])
        elif (curve_degree==2):
            xm=3*(start[0]+stop[0])/4
            if (start[0]<stop[0]):
                ym=start[1]
            else:
                ym=stop[1]
            coeff=np.dot(np.linalg.inv(np.array([[(start[0] - xm) ** 2,start[0] - xm],[(stop[0] - xm) ** 2,stop[0] - xm]])),np.array([[start[1] - ym],[stop[1] - ym]]))
            a=coeff[0]
            b=coeff[1]
            y_ref = a * (x_ref - xm) ** 2 + b*(x_ref- xm) + ym
    except:
        if (start[1]<=stop[1]):
            y_ref=np.arange(start[1],stop[1],spacing)
        else:
            y_ref=np.arange(start[1],stop[1],-spacing)
        #print(len(x_ref))
        x_ref=[0]*len(y_ref)
        if (curve_degree==1):
            x_ref=[start[0]]*len(y_ref)
        elif (curve_degree==2):
            xm=3*(start[0]+stop[0])/4
            if (start[0]<stop[0]):
                ym=start[1]
            else:
                ym=stop[1]
            coeff=np.dot(np.linalg.inv(np.array([[(start[0] - xm) ** 2,start[0] - xm],[(stop[0] - xm) ** 2,stop[0] - xm]])),np.array([[start[1] - ym],[stop[1] - ym]]))
            a=coeff[0]
            b=coeff[1]
            y_ref = a * (x_ref - xm) ** 2 + b*(x_ref- xm) + ym
    wp_pos = np.array([x_ref,y_ref])
    return wp_pos

def wrap_angle_to_pmpi(angle):
    """
    Wraps an angle to the range [-pi, pi].
    """
    wrapped_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return wrapped_angle

def unwrap_angle(prev,curr):
    return prev+wrap_angle_to_pmpi(curr-prev)

def check_for_wp_segment_switch(self, wp_segment, d_0wp):
    """
    Checks if a switch should be made from the current to the next waypoint segment.
    
    Args:
        wp_segment: 2D vector describing the distance from waypoint i to i + 1 in the current segment.
        d_0wp: 2D distance vector from state to waypoint i + 1.
    
    Returns:
        bool: If the switch should be made or not.
    """
    wp_segment = self.normalize_vec(wp_segment)
    d_0wp_norm = np.linalg.norm(d_0wp)
    d_0wp = self.normalize_vec(d_0wp)
    
    segment_passed = np.dot(wp_segment,d_0wp) < np.cos(np.deg2rad(self.pass_angle_threshold))
    #segment_passed = segment_passed or d_0wp_norm <= self.R_a
    return segment_passed

def normalize_vec(self, vec):
    """
    Normalizes a vector.
    """
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm