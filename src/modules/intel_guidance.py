import numpy as np
import json
import cvxpy as cp
from scipy.integrate import odeint, solve_ivp
import os
import networkx as nx
   
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
    Tenc=gparams[2]<=2*myvessel.ontology.L[0]
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
    # if (myvessel.type=="Other4"):
    #     print("Am I in an inland waterway?", T_inland)
    #     print("Should I give-way?", Tent_GW_inland)
    #     print("Did someone enter my encounter radius?", Tenc )
    #     print("Is it a head-on?",Thdn)
    #     print("Does BPR make me a give-way?",Tbpr)
    #     print("Does stb not hold?", not Tstb)
    if (Tent_GW and (not T_inland)) or ((Tent_GW_inland) and T_inland) or (Tent_EM):
        myvessel.role='Give-way'
    elif (Text_GW or Text_EM):
        myvessel.role='Stand-On'
            

def Guidance_Reasoner(time, myvessel, other_vessels, ontology, sim_scale,current_script_dir,map=None):
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
    #vessel_index_in_radius=0
    vessels_in_radius=[]
    if (len(enc_params)>=1): # there are target ships to avoid
        for ii in range(0,len(enc_params)): 
            Finite_State_Machine(enc_params[ii], dsafe, myvessel, time, sim_scale)
            #vessel_index_in_radius=ii
            vessels_in_radius.append(other_vessels[ii])
    else:
        myvessel.scenario="Safe"
        myvessel.role="Stand-On"
    # ================================================
    # 4. Calculation of own path deviation
    #=================================================
    map_segment_closest_to_vessel = closest_segment(map, myvessel, sim_scale) 
    Dx1=np.zeros(len(myvessel.route[0])).tolist()
    Dy1=np.zeros(len(myvessel.route[0])).tolist()
    Droute_ref_own=(np.zeros(len(myvessel.route[0])).tolist(),np.zeros(len(myvessel.route[1])).tolist())
    # Calculate distance between vessel and boundaries d_{WEj} and check against safe distance 
    #print(myvessel.role)
    if (myvessel.role=="Stand-On"):
        pass
    else:
        # if (reg_set=="BPR"):
        route_points=[np.array([myvessel.route[0][ii], myvessel.route[1][ii]]) for ii in range(0,len(myvessel.route[0]))]
        distance_from_current=[np.sqrt((route_points[jj][0] - myvessel.ontology.yx[-1])**2 + (route_points[jj][1] - myvessel.ontology.yy[-1])**2 ) for jj in range(0,len(route_points))]
        pos=np.argmin(distance_from_current)
        #print(myvessel.type, ' has route ', route_points)
        #print(myvessel.type, ' has position ', [myvessel.y[3], myvessel.y[4]])
        #print("Vessel just passed waypoint", pos)
        #Dx1, Dy1= generate_path_envelope(myvessel, other_vessels[vessel_index_in_radius],path_def,dsafe,1,pos,Dx1,Dy1,current_script_dir)
        Dx1, Dy1= generate_path_envelope2(myvessel, vessels_in_radius, map, path_def, dsafe,pos, Dx1, Dy1, map_segment_closest_to_vessel, current_script_dir)
        Droute_ref_own=(np.asarray(Dx1),np.asarray(Dy1))
        # else:
        #     route_points=[np.array([myvessel.route[0][ii], myvessel.route[1][ii]]) for ii in range(0,len(myvessel.route[0]))]
        #     distance_from_current=[np.sqrt((route_points[jj][0] - myvessel.y[3])**2 + (route_points[jj][1] - myvessel.y[4])**2 ) for jj in range(0,len(route_points))]
        #     two_lowest=np.partition(distance_from_current, 1)[0:2]
        #     sum_two_lowest=np.sum(two_lowest)
        #     print("Two lowest (sum)",sum_two_lowest)
        #     pos=np.argmin(distance_from_current)
        #     #print(myvessel.type, ' has route ', route_points)
        #     #print(myvessel.type, ' has position ', [myvessel.y[3], myvessel.y[4]])
        #     #print("Vessel just passed waypoint", pos)
        #     Dx1, Dy1= generate_path_envelope(myvessel, other_vessels[vessel_index_in_radius],path_def,dsafe,0,pos,Dx1,Dy1,current_script_dir)
        #     Droute_ref_own=(np.asarray(Dx1),np.asarray(Dy1))
    myvessel.droute=Droute_ref_own
    myvessel.Dx=Dx1
    myvessel.Dy=Dy1          
    #return Droute_ref_own, Usafe, dsafe, myvessel.scenario,other_vessels[vessel_index_in_radius]
    return Droute_ref_own, Usafe, dsafe, myvessel.scenario, vessels_in_radius, map_segment_closest_to_vessel

def closest_segment(polyline, vessel,sim_scale):
    point=(sim_scale*vessel.ontology.yx[-1],-sim_scale*vessel.ontology.yy[-1])
    P = np.array(point)
    best_dist = float("inf")
    best_segment = None
    best_projection = None
    best_t = None

    for i in range(len(polyline) - 1):
        A = np.array(polyline[i])
        B = np.array(polyline[i+1])

        AB = B - A
        AP = P - A

        # projection parameter t
        t = np.dot(AP, AB) / np.dot(AB, AB)

        # clamp to segment
        t_clamped = max(0, min(1, t))

        # projected point
        Q = A + t_clamped * AB

        # distance
        dist = np.linalg.norm(P - Q)

        # update best
        if dist < best_dist:
            best_dist = dist
            best_segment = (tuple(A), tuple(B))
            best_projection = tuple(Q)
            best_t = t_clamped

    return best_segment

def generate_path_envelope2(myvessel, other_vessels, map, path_def, dsafe,pos, Dx_old, Dy_old, map_segment_closest_to_vessel, current_script_dir):
    Dx = np.zeros(len(myvessel.route[0])).tolist()
    Dy = np.zeros(len(myvessel.route[1])).tolist()

    rho_s = myvessel.ontology.B[0] / 2
    rho_p = myvessel.ontology.L[0] / 2

    xref = myvessel.route[0]
    yref = myvessel.route[1]

    # bank geometry placeholders (you must define these properly)
    # Example structure:
    xBj = map_segment_closest_to_vessel[0][0]   # list or array
    yBj = map_segment_closest_to_vessel[0][1]   # list or array
    delta_xBj = map_segment_closest_to_vessel[1][0]-map_segment_closest_to_vessel[0][0]
    delta_yBj = map_segment_closest_to_vessel[1][1]-map_segment_closest_to_vessel[0][1]
    w=np.abs(myvessel.distance_from_bank1)+np.abs(myvessel.distance_from_bank2)
    s_w = 0.2  # tuning weight (adjust based on your formulation)

    k = pos
    while k < path_def - 1:

        # decision variables
        dx = cp.Variable()
        dy = cp.Variable()
        delta = cp.hstack([dx, dy])
        constraints = []

        # =========================
        # Consideration of other vessels
        # =========================
        for vessel in other_vessels:
            xo = vessel.ontology.yx[-1]
            yo = vessel.ontology.yy[-1]
            psi_o = vessel.ontology.ypsi[-1]
            rho_q = vessel.ontology.L[0] / 2
            n = np.array([-np.sin(psi_o), np.cos(psi_o)])

            rhs = (rho_s + rho_p + rho_q) + cp.abs((yo - yref[k+1]) * np.cos(psi_o) +
                    (xref[k+1] - xo) * np.sin(psi_o))
            lhs = n @ delta

            constraints.append(lhs >= rhs)
        # =========================
        # Consideration of Infrastructure
        # =========================
        cross = delta_xBj * dy - delta_yBj * dx

        geom_term = cp.abs(
            delta_xBj * (yBj - yref[k+1]) -
            (xBj - xref[k+1]) * delta_yBj
        )

        norm_B = cp.sqrt(delta_xBj**2 + delta_yBj**2)

        constraints.append(
            cross <= (1 - s_w) * w * norm_B + geom_term
        )
        constraints.append(
            cross >= -(1 - s_w) * w * norm_B + geom_term  
        )
        # constraints.append(
        #     cross >= s_w * w * norm_B + geom_term
        # )
        # constraints.append(
        #     cross <= -s_w * w * norm_B + geom_term
        # )
        # =========================
        # Envelope preference
        # =========================
        constraints.append(-(yref[k+1] - yref[k]) * dx +(xref[k+1] - xref[k]) * dy <= 0)

        # =========================
        # Solve problem
        # =========================
        problem = cp.Problem(cp.Minimize(dx**2 + dy**2), constraints)

        problem.solve()

        if dx.value is not None and dy.value is not None:
            Dx[k+1] = dx.value
            Dy[k+1] = dy.value
        else:
            Dx[k+1] = Dx_old[k+1]
            Dy[k+1] = Dy_old[k+1]

        k += 1

    return Dx, Dy

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
    if d_route is not None:
        wp_pos = (vessel.route[0]+d_route[0], vessel.route[1]+d_route[1])
    else:
        wp_pos=(vessel.route[0], vessel.route[1])
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

def compute_LOS_ref(self, d_route, current_script_dir):
    chi_rate_max = np.pi / 90  # [rad/s]
    Delta = 5*self.ontology.L[0]

    # --- Current state ---
    x = self.ontology.yx[-1]
    y = self.ontology.yy[-1]

    p = np.array([x, y])

    # --- Waypoints ---
    if d_route is None:
        wp_x = self.route[0]
        wp_y = self.route[1]
    else:
        wp_x = self.route[0] + d_route[0]
        wp_y = self.route[1] + d_route[1]

    n_wps = wp_x.shape[0]

    while self.wp_idx < n_wps - 1:

        p0 = np.array([wp_x[self.wp_idx], wp_y[self.wp_idx]])
        p1 = np.array([wp_x[self.wp_idx + 1], wp_y[self.wp_idx + 1]])

        d = p1 - p0
        seg_len_sq = np.dot(d, d)

        if seg_len_sq < 1e-6:
            self.wp_idx += 1
            continue

        proj = np.dot(p - p0, d)

        if proj > seg_len_sq:
            self.wp_idx += 1
        else:
            break

    if self.wp_idx < n_wps - 1:
        p0 = np.array([wp_x[self.wp_idx], wp_y[self.wp_idx]]) # segment
        p1 = np.array([wp_x[self.wp_idx + 1], wp_y[self.wp_idx + 1]])

        d = p1 - p0
        d_norm = np.linalg.norm(d)

        if d_norm < 1e-6:
            return self.psi_d_prev

        d_unit = d / d_norm

        alpha = np.arctan2(d_unit[1], d_unit[0])
        y_e = d_unit[0] * (p[1] - p0[1]) - d_unit[1] * (p[0] - p0[0])
        self.e = y_e

        psi_d_raw = alpha - np.arctan2(y_e, Delta)
        psi_d_raw = wrap_angle_to_pmpi(psi_d_raw)
        psi_diff = wrap_angle_to_pmpi(psi_d_raw - self.psi_d_prev)
        psi_rate_max = chi_rate_max * self.timestep
        psi_diff = np.clip(psi_diff, -psi_rate_max, psi_rate_max)
        psi_d = wrap_angle_to_pmpi(self.psi_d_prev + psi_diff)
        self.psi_d_prev = psi_d
    else:
        psi_d = wrap_angle_to_pmpi(self.psi_d_prev)
    self.chi.append(psi_d)
    return psi_d
    

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

#==========================================
# Global planner [offline]
#==========================================
def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def intersect(A, B, C, D):
    return (ccw(A, C, D) != ccw(B, C, D)) and (ccw(A, B, C) != ccw(A, B, D))

def is_line_clear(p1, p2, segments):
    for seg_start, seg_end in segments:
        if (np.array_equal(p1, seg_start) or np.array_equal(p1, seg_end) or
            np.array_equal(p2, seg_start) or np.array_equal(p2, seg_end)):
            continue
        if intersect(p1, p2, seg_start, seg_end):
            return False
    return True

def generate_safety_vertices(points, rho):
    n = len(points)
    safety_points = np.zeros_like(points)
    
    for i in range(n):
        # Get the current vertex, the previous one, and the next one
        p_curr = points[i]
        p_prev = points[(i - 1) % n]
        p_next = points[(i + 1) % n]
        
        # Calculate normalized edge direction vectors
        v1 = p_curr - p_prev
        v2 = p_next - p_curr
        n1 = np.array([-v1[1], v1[0]]) / np.linalg.norm(v1) # Left normal to edge 1
        n2 = np.array([-v2[1], v2[0]]) / np.linalg.norm(v2) # Left normal to edge 2
        
        # Bisector normal vector (average of the two edge normals)
        bisector = n1 + n2
        bisector_norm = np.linalg.norm(bisector)
        
        if bisector_norm < 1e-6:
            # Collinear points fallback
            safety_points[i] = p_curr + n1 * rho
            continue
            
        bisector_dir = bisector / bisector_norm
        
        # Compute exact scale factor so the perpendicular clearance is exactly rho
        # cos(theta) calculation via dot product
        scale = rho / np.dot(bisector_dir, n1)
        
        # Push the point inward along the calculated vector trajectory
        safety_points[i] = p_curr + bisector_dir * scale
        
    return safety_points

def safe_resample_route(path_points, step_distance, boundary_segments):
    """
    Resamples a path while strictly forcing waypoints at tight bends 
    to prevent any boundary intersection shortcuts.
    """
    path_points = np.array(path_points, dtype=np.float64)
    resampled_points = [path_points[0]]
    
    current_pos = np.copy(path_points[0])
    target_node_idx = 1
    
    while target_node_idx < len(path_points):
        target_node = path_points[target_node_idx]
        
        # Vector and distance to the next actual path cornerstone
        to_target = target_node - current_pos
        dist_to_target = np.linalg.norm(to_target)
        
        if dist_to_target < 1e-6:
            target_node_idx += 1
            continue
            
        # If the milestone corner is closer than our stride length, 
        # we MUST step directly onto the corner to prevent clipping.
        if dist_to_target <= step_distance:
            current_pos = np.copy(target_node)
            resampled_points.append(current_pos)
            target_node_idx += 1
        else:
            # Take a standard step toward the target
            step_vector = (to_target / dist_to_target) * step_distance
            next_potential_pos = current_pos + step_vector
            
            # Double check: Does this tiny micro-step step out of bounds?
            is_safe = True
            for seg_start, seg_end in boundary_segments:
                # Ignore shared corner endpoints
                if (np.array_equal(current_pos, seg_start) or np.array_equal(current_pos, seg_end) or
                    np.array_equal(next_potential_pos, seg_start) or np.array_equal(next_potential_pos, seg_end)):
                    continue
                if intersect(current_pos, next_potential_pos, seg_start, seg_end):
                    is_safe = False
                    break
            
            if is_safe:
                current_pos = next_potential_pos
                resampled_points.append(current_pos)
            else:
                # Safety fallback: If it cuts a wall, drop anchor at the original corner vertex
                current_pos = np.copy(target_node)
                resampled_points.append(current_pos)
                target_node_idx += 1
        return resampled_points

def global_planner(boundary,anchors,rho,distance_btw_wp):
    boundary=np.array([list(b) for b in boundary])
    safety_vertices = generate_safety_vertices(boundary, rho)
    num_v = len(boundary)
    original_segments = [
        (boundary[i], boundary[(i + 1) % num_v]) for i in range(num_v)
    ]

    # Master node map: 0=Start, 1=End, 2+=Safety Turn vertices
    nodes = anchors + list(safety_vertices)
    num_nodes = len(nodes)

    G = nx.Graph()
    for i, pos in enumerate(nodes):
        G.add_node(i, pos=pos)

    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            # Paths must not cross the ORIGINAL physical boundary lines
            if is_line_clear(nodes[i], nodes[j], original_segments):
                dist = np.linalg.norm(nodes[i] - nodes[j])
                G.add_edge(i, j, weight=dist)

    # Solve the graph
    try:
        path_indices = nx.shortest_path(G, source=0, target=1, weight='weight')
        route=[G.nodes[idx]['pos'] for idx in path_indices]
        route_x = [pt[0] for pt in route]
        route_y = [pt[1] for pt in route]
        return route_x, route_y
        #return  safe_resample_route(route,distance_btw_wp,original_segments)
    except nx.NetworkXNoPath:
        print("No safe path could be found through these coordinates.")
        return []