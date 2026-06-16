from math import inf
from scipy.integrate import odeint, solve_ivp
import numpy as np
import matplotlib.pyplot as plt
import pygame
import pandas as pd
import sys
sys.path.insert(1, 'src/')
from traffic_planner import *
from hydrostatics import *
from owlready2 import default_world
import copy
import time
#===============================================
# Classes
#===============================================
class vessel:
    def __init__(self,name,ontology,x0):
        self.state=x0
        self.extended_state=[x for i,x in enumerate(x0) if i!=5]
        self.extended_state=self.extended_state+[0]*15
        self.y=x0
        self.e=[]
        self.ehat=[]
        self.f=[]
        self.alpha=0
        self.beta=[]
        self.tf=[]
        self.psihat=[]
        self.betahat=[]
        self.uhat=[]
        self.fhatu=[]
        self.fhatx=[]
        self.fhatpsi=[]
        self.d0=[0,0,0,0,0,0,0]
        self.d=[0,0,0,0,0,0,0]
        self.Td=[inf]*6
        self.Omega=[0,0,0,0,0,0]
        self.Fhat_history=pd.DataFrame({'f1' : [],'f2' : [],'f3' : [],'f4' : [],'f5' : [],'f6' : []})
        self.z=0
        self.u=[0]#[[[300/60],[0],[0.328]]]#[0]#[[0,150/60]]
        self.head_on_upperbound=[]
        self.head_on_lowerbound=[]
        self.ratio_head_on=[]
        self.cond1=[]
        self.chi=[0]
        self.tau=0.02
        self.usafe=[]
        self.time=0
        self.timestep=1
        self.times=[self.time]
        self.route=None
        self.role="Stand-On"
        self.error=0
        self.arrived=0
        self.ratio_time=[]
        self.MPC=None
        self.timehat=[]
        self.simulator=None
        self.estimator=None
        self.scenario="Safe"
        self.positioning_noise=[1,1,np.radians(2)]
        self.droute=(np.asarray([0]*100),np.asarray([0]*100))
        self.wp_idx = 0  # Initialize waypoint index
        self.psi_d_prev = x0[-1]  # Initialize previous desired heading
        self.yi_prev=0
        self.sum_error = 0  # Initialize sum of heading errors
        self.error_old = 0  # Initialize previous heading error
        self.pass_angle_threshold = 90 # Angle threshold for passing a waypoint segment
        self.R_a = 5 
        self.control = 0  # Initialize delta
        self.aggregated_error=0
        self.measurement_noise=0
        self.type=name
        if (self.type=="Own"):
            self.ontology=ontology.Own
        elif (self.type=="Other1"):
            self.ontology=ontology.Other1
        elif (self.type=="Other2"):
            self.ontology=ontology.Other2
        elif (self.type=="Other3"):
            self.ontology=ontology.Other3
        else:
            self.ontology=ontology.Other4
            #self.calculate_derivatives()
        # try:
        #     self.hydrostatics=pd.read_excel('hydrostatic_tables_with_heel_trim.xlsx')
        # except:
        #     try:
        #         self.hydrostatics=pd.read_excel('hydrostatic_tables_with_heel_trim.xlsx')
        #     except:
        #         #print('Filename not found. Vessel will be initialised but without hydrostatics')
        #         self.hydrostatics=[]
        self.distance_from_bank1=10**(-3)
        self.distance_from_bank2=10**(-3)
    #New
    def include_sensor_noise(self,noise):
        self.measurement_noise=noise
    def include_sensor_faults(self,f,tf):
        self.f=f
        self.tf=tf
        self.Td=tf
    def dynamic_ontology_update(self,mystate,reference,mytime,start_time):
        self.ontology.x.extend([float(x1) for x1 in mystate[3]])
        self.ontology.y.extend([float(y1) for y1 in mystate[4]])
        t1_len = len(mytime.tolist())
        self.ontology.yx.extend([float(x2+self.positioning_noise[0]*np.random.uniform(-1,1,size=None)) for x2 in mystate[3]])
        self.ontology.yy.extend([float(y2+self.positioning_noise[1]*np.random.uniform(-1,1,size=None)) for y2 in mystate[4]])
        self.ontology.ypsi.extend([float(p1+self.positioning_noise[2]*np.random.uniform(-1,1,size=None)) for p1 in mystate[-1]])
        self.ontology.psi.extend([float(p2) for p2 in mystate[-1]])
        self.ontology.psi_d.extend([float(reference)] * t1_len)
        usq = [float(i)**2 for i in mystate[0]]
        vsq = [float(i)**2 for i in mystate[1]]
        Usq = [sum(x) for x in zip(usq, vsq)]
        self.ontology.SOG.extend([float(np.sqrt(i)*(1.0+self.measurement_noise*np.random.uniform(-1,1,size=None))) for i in Usq])
        self.ontology.rudder_angle.extend([float(self.u[-1])] * t1_len)
        self.ontology.starboard_distance.extend([float(self.distance_from_bank1)] * t1_len)
        self.ontology.port_distance.extend([float(self.distance_from_bank2)] * t1_len)
        self.ontology.time.extend([float(tt + start_time) for tt in mytime])
        self.time=start_time
    def check_for_wp_segment_switch(self, wp_segment, d_0wp):
        return check_for_wp_segment_switch(self, wp_segment, d_0wp)

    def normalize_vec(self, vec):
        return normalize_vec(self, vec)

    def wrap_angle_to_pmpi(self, angle):
        return wrap_angle_to_pmpi(self, angle)
    
    def PID_controller(self, psi_d, dt):
        return PID_controller(self, psi_d, dt)
    
    def MPC_controller(self, psi_d):
        self.MPC=MPC_controller(self,psi_d)
        return MPC_controller(self, psi_d)
    def make_MPC_step(self):
         return self.MPC.make_step(np.array(self.y[3:5]+self.y[6:]+self.y[0:3]).reshape(-1,1))
    def reset(self):
        self.state=[2.5,0,0,0,0,0,0]
        self.u=[[[300/60],[0],[0.328]]]#[[0,0]]
        self.y=[2.5,0,0,0,0,0,0]
        self.z=0
    def calculate_derivatives(self):
        self.ontology.Yvdot=(1025*self.ontology.L[0]**(3)/2)*(-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1+0.16*self.ontology.Cb[0]*self.ontology.B[0]/self.ontology.Td[0]-5.1*(self.ontology.B[0]/self.ontology.L[0])**2))
        self.ontology.Nvdot=(1025*self.ontology.L[0]**(4)/2)*(-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(-0.0041*self.ontology.B[0]/self.ontology.Td[0]+1.1*(self.ontology.B[0]/self.ontology.L[0])))
        self.ontology.Yv=(-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1+0.4*self.ontology.Cb[0]*self.ontology.B[0]/self.ontology.Td[0]))
        self.ontology.Yrdot=(1025*self.ontology.L[0]**(4)/2)*(-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1+0.16*self.ontology.Cb[0]*self.ontology.B[0]/self.ontology.Td[0]-5.1*(self.ontology.B[0]/self.ontology.L[0])**2))
    def sensor_output(self):
        for ii in range(0,len(self.state)):
            if ii<3:
                self.y[ii] = self.state[ii]*(1.0+self.measurement_noise*np.random.uniform(-1,1,size=None)) # relative noise on velocities, uniform
                # self.y.append(state_output[i]*(1.0+self.measurement_noise*np.clip(np.random.normal(0,1),-1,1))) # relative noise on velocities, normal
            else:
                self.y[ii] = self.state[ii]+self.positioning_noise[ii-3]*np.random.uniform(-1,1,size=None) # absolute noise on coordinates, uniform

    #def calculate_LOS_ref_vs_enabled(self,wp_pos):
        
    def update_derivatives_inland(self,envr):
        k=2*self.ontology.Td[0]/self.ontology.L[0]
        ke=k/((self.T/(2*envr.H))*k+((np.pi*self.ontology.Td[0])/(2*envr.H))/ np.tan((np.pi*self.ontology.Td[0])/(2*envr.H))**0.7)
        self.ontology.Yr=np.pi/4*ke
        self.ontology.Nr=-0.54*ke+ke**2
        ke=k/((self.ontology.Td[0]/(2*envr.H))*k+((np.pi*self.ontology.Td[0])/(2*envr.H))/ np.tan((np.pi*self.ontology.Td[0])/(2*envr.H))**2.3)
        self.ontology.Yv=-np.pi/2*ke-1.4*self.ontology.Cb[0]*self.ontology.B[0]/self.ontology.L[0]
        ke=k/((self.ontology.Td[0]/(2*envr.H))*k+((np.pi*self.ontology.Td[0])/(2*envr.H))/ np.tan((np.pi*self.ontology.Td[0])/(2*envr.H))**1.7)
        self.ontology.Nv=-ke
    def calculate_forces(self,v):
        """
            Calculate the forces acting on the hull, propeller, and rudder of a ship, as well as the bank effects.
            Parameters:
            -----------
            v : list or array-like
                A list or array containing the non-dimensionalised velocity and position components, with:
                - v[0] is the surge velocity (forward speed)
                - v[1] is the sway velocity (sideways speed)
                - v[2] is the yaw rate (rotational speed)
                - v[3] is the x-coordinate of the ship
                - v[4] is the y-coordinate of the ship
                - v[5] is the z-coordinate of the ship
                - v[6] is the yaw angle of the ship

            Constants:
            ----------
            U : float
                Vessel velocity
            Fn : float
                Froude number
            Dk : float
                Drag coefficient
            Sw : float
                Wetted surface area
            bm : float  
                Vessel angle of drift at CG
            R0 : float
                Non-dimensionalised resistance coefficient
            tP : float
                Propeller thrust deduction factor
            Np : float
                Propeller speed in rps
            wp0 : float
                Propeller nominal wake fraction
            C1 : float
                Propeller wake change characteristic versus bp
            Dp : float
                Propeller diameter in m
            lp : float
                Propeller center of acting force
            p : float
                Propeller pitch
            b : float
                Vessel angle of drift at origin 
            bp : float
                Geometrical inflow angle to the propeller
            wp : float
                Propeller wake fraction
            Jp : float
                Propeller advance ratio
            KT : float
                Propeller thrust coefficient (function of Jp)
            tR : float
                Steering resistance deduction factor
            aH : float
                Rudder force increase factor
            lR : float
                Position of  an additional lateral force component acting on the rudder
            xH : float
                Position of  an additional lateral force component acting on the rudder
            gR : float
                flow straightening coefficient
            AR : float
                Rudder area
            e : float
                ratio of wake fraction at rudder position to that at propeller position
            k : float
                Experimental constant
            w : float
                Rudder efficiency factor
            s : float
                Propeller slip ratio
            xR : float
                Position of the rudder
            delta : float
                Rudder angle in degrees
            delta_rad : float
                Rudder angle in radians
            uP : float
                Propeller advance velocity
            fa : float
                Gradient of the normal force coefficient with respect to rudder angle
            bR : float
                Drift angle at the rudder
            vR : float
                Sway component of the rudder inflow velocity
            eta : float
                Rudder-propeller ratio coefficient
            uR : float
                Surge component of the rudder inflow velocity
            aR : float
                Rudder inflow angle
            UR : float
                Rudder inflow velocity
            FN : float
                Rudder normal force
            xR : float
                x-coordinate point where FN acts on the rudder
            yB3 : float
                Non-dimensional ship-bank distance
            a_ikH : list
                Coefficients for the ship-bank effect sway force
            b_ikH : list
                Coefficients for the ship-bank effect yaw moment
            a_ikHP : list
                Coefficients for the ship-bank effect sway force
            b_ikHP : list
                Coefficients for the ship-bank effect yaw moment
            Returns:
            --------
            tuple
                A tuple containing the following non-dimensionalised forces and moments:
                - XH : float
                    Hull surge force
                - YH : float
                    Hull sway force
                - NH : float
                    Hull yaw moment
                - XP : float
                    Propeller thrust force
                - XR : float
                    Rudder surge force
                - YR : float
                    Rudder sway force
                - NR : float
                    Rudder yaw moment
                - YB : float
                    Bank effect sway force
                - NB : float
                    Bank effect yaw moment
        """
        #================================================
        # 1. Constants
        #================================================
        R0=0.0428                    #[checked]
        t=0.326                      #[checked]
        wp0=0.576                    #[checked]
        C1=0.98
        Dp=1.8                       #[checked]
        lp=(self.ontology.L[0]/2+self.ontology.xg[-1])/(self.ontology.L[0])
        aH=0.418                     #[checked]
        lR=-1.113                    #[checked]
        tR=0.055                     #[checked]
        xH=-0.189*(self.ontology.L[0])                    #[checked]
        gR=0.293                     #[checked]
        AR=4
        e=1.823                      #[checked]
        k=0.6/e                      #[checked]
        w=0.3
        xR=-self.ontology.L[0]/2
        wb=4 # Bank width
        ys=self.distance_from_bank1
        yp=self.distance_from_bank2
        yB3 = 0.5*self.ontology.B[0]*(1/yp+1/ys)  # Non-dimensional ship-bank distance
        a_ikH=[-0.461+0.368/self.ontology.Cb[0],-0.173+0.0114*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0],0.00634-0.00046*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0], 1.36-0.0658*self.ontology.L[0]/self.ontology.Td[0],0.257-0.0896*self.ontology.B[0]/self.ontology.Td[0],-0.0191+0.00634*self.ontology.B[0]/self.ontology.Td[0]]
        b_ikH=[-0.119+2.11*self.ontology.Td[0]/self.ontology.L[0],-0.0354+0.195*self.ontology.B[0]/self.ontology.L[0],2.29*10**(-4)-6.92*10**(-5)*self.ontology.B[0]/self.ontology.Td[0], 0.134-2.35*self.ontology.Td[0]/self.ontology.L[0],-4.23*10**(-2)+2.93*10**(-3)*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0],1.23*10**(-3)-8.53*10**(-5)*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0]]
        a_iKP=[0.1 - 0.0524 * self.ontology.Td[0] / self.ontology.B[0], -0.0087 - 0.0386 * self.ontology.B[0] / self.ontology.L[0], -0.000105 - 0.000000923 * self.ontology.L[0] / self.ontology.B[0], 0.0592 - 0.0211 * self.ontology.B[0] / self.ontology.Td[0], -0.000282 - 0.000242 * self.ontology.L[0] / self.ontology.B[0], 0.000232 - 0.0000198 * self.ontology.L[0] / self.ontology.B[0]]
        b_iKP=[-0.00495 + 0.125 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.L[0], 0.00129 + 0.00537 * self.ontology.Td[0] / self.ontology.B[0], -0.000268 + 0.000878 * self.ontology.Td[0] / self.ontology.B[0],0.00381 - 0.0129 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.B[0], 0.00344 - 0.0011 * self.ontology.B[0]/ self.ontology.Td[0], 0.000488 - 0.00152 * self.ontology.Td[0] / self.ontology.B[0]]
        a_ikHP=[-0.237 + 0.422 * self.ontology.Td[0] / self.ontology.B[0], 0.0289 - 0.0022 * self.ontology.L[0] / self.ontology.B[0], -0.00657 + 0.00118 * self.ontology.L[0] / self.ontology.B[0], 0.419 - 0.0479 * self.ontology.L[0] / self.ontology.B[0], -0.0449 + 0.00998 * self.ontology.L[0] / self.ontology.B[0], 0.0106 + 0.0424 * self.ontology.Td[0] / self.ontology.B[0]]
        b_ikHP=[-0.289 + 5.71 * self.ontology.Td[0] / self.ontology.L[0], -0.122 + 0.353 * self.ontology.Td[0] / self.ontology.B[0], 0.00787 - 0.0233 * self.ontology.Td[0] / self.ontology.B[0],-0.0933 + 0.0157 * self.ontology.L[0] / self.ontology.B[0], 0.26 - 0.795 * self.ontology.Td[0] / self.ontology.B[0], -0.0199 + 0.0598 * self.ontology.Td[0] / self.ontology.B[0]]
        # Td_affected=self.ontology.Td[0]+self.z
        #================================================
        # 2. Tunable parameters by controller
        #================================================
        delta=self.u[-1]#[-1][1][0]
        # print("delta is ", delta)
        #c=0.364/(150/60)**2
        p=0.328#self.u[-1][2][0]#0.328
        Np=300/60#self.u[-1][0][0]#300/60#500/60 #self.u[-1][1]
        #================================================
        # 3. Problem defined variables
        #================================================
        U=np.sqrt((v[0])**2+(v[1])**2)
        s=1-U/(Np*Dp)                #[checked]
        #print("Np is ", Np)
        #================================================
        # 4. Hull forces calculation
        #================================================
        Fn=v[0]/np.sqrt(9.81*self.ontology.L[0])
        bm=-np.arctan((v[1]-self.ontology.xg[0]*v[2])/v[0])
        r_nd=v[2]*self.ontology.L[0]/U # Non-dimensional yaw rate
        try:
            XH=-R0*np.cos(bm)**2+self.ontology.Xbb[0]*bm**2+self.ontology.Xbr[0]*bm*r_nd+self.ontology.Xrr[0]*r_nd**2
            YH=self.ontology.Yb[0]*bm+self.ontology.Yr*r_nd+self.ontology.Ybbb[0]*bm**3+self.ontology.Ybbr[0]*bm**2*r_nd+self.ontology.Ybrr[0]*bm*r_nd**2+self.ontology.Yrrr[0]*r_nd**3
            NH=self.ontology.Nb[0]*bm+self.ontology.Nr*r_nd+self.ontology.Nbbb[0]*bm**3+self.ontology.Nbbr[0]*bm**2*r_nd+self.ontology.Nbrr[0]*bm*r_nd**2+self.ontology.Nrrr[0]*r_nd**3
        except:
            XH=self.ontology.Xu*(v[0]-U)/U #+Yudot*udot
            YH=self.ontology.Yv*v[1]/U+self.ontology.Yr*v[2]*self.ontology.L[0]/U #+Yvdot*vdot+Yrdot*rdot
            NH=self.ontology.Nv*v[1]/U+self.ontology.Nr*v[2]*self.ontology.L[0]/U #+Yvdot*vdot+Yrdot*rdot
            #print("XH",XH)
            #print("YH",YH)
            #print("NH",NH)
        #================================================
        # 4. Propeller force calculation
        #================================================
        b=-np.arctan((v[1])/v[0])
        bp=b-lp*r_nd
        wp=wp0*np.exp(-C1*bp**2)
        Jp=v[0]*(1-wp)/(Np*Dp)
        KT=-0.326*p*Jp-0.2005*Jp+0.5234*p-0.0398
        XP=(2*(1-t)*1025*Np**2*Dp**4*KT)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)
        #================================================
        # 5. Rudder forces calculation
        #================================================
        #delta_rad = delta#*np.pi/180  # Convert to radians
        uP=v[0]*(1-s)
        fa = 6.13/(1+2.25)  # Gradient of the normal force coefficient with respect to rudder angle
        bR=b-lR*r_nd
        vR=v[1]*gR*bR
        eta=Dp/(2)
        uR=e*uP/(1-s)*np.sqrt(1-2*(1-eta*k)*s+(1-eta*k*(2-k))*s**2)
        aR=delta-np.arctan(vR/uR)
        UR=np.sqrt(uR**2+vR**2)
        FN=(0.5*1025*AR*UR**2*fa*np.sin(aR))
        XR=-(1-tR)*2*FN*np.sin(delta)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)
        YR=-(1+aH)*2*FN*np.cos(delta)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)
        NR=-(xR+aH*xH)*2*FN*np.cos(delta)/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)
        #================================================
        # 6. Bank effects calculation
        #================================================
        #print(U)
        #print(XP)
        Uref=np.sqrt(np.abs((1025*0.25*self.ontology.L[0]*self.ontology.Td[0]*U**2)*XP/(1025*np.pi*Dp**2/8))) # Reference velocity
        h_eff=1.2*self.ontology.Td[0]#cal_depth([v[3],v[4]],self.depths_data)-self.z
        T_hT=self.ontology.Td[0]/(h_eff-self.ontology.Td[0])
        Y_BH = sum(a * np.sign(yB3) * np.abs(yB3)**(i//3+1) *T_hT**(i%3)  for i, a in enumerate(a_ikH)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0] * (v[0]**2)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)
        N_BH = sum(b * np.sign(yB3) * np.abs(yB3)**(i//3+1)*T_hT**(i%3)  for i, b in enumerate(b_ikH)) * 0.5*1025*self.ontology.L[0]**2 * self.ontology.Td[0] * (v[0]**2)/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)
        Y_BP = sum(p * np.sign(yB3) * np.abs(yB3)**(i//3+1)*T_hT**(i%3)  for i, p in enumerate(a_iKP)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0]*(Uref**2)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)
        N_BP = sum(q * np.sign(yB3) * np.abs(yB3)**(i//3+1)*T_hT**(i%3)  for i, q in enumerate(b_iKP)) * 0.5*1025*self.ontology.L[0]**2 * self.ontology.Td[0]*(Uref**2)/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)
        YB = (sum(hp * np.sign(yB3) * np.abs(yB3)**(i//3+1)*T_hT**(i%3)  for i, hp in enumerate(a_ikHP)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0]*Fn*Uref**2)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)
        NB = (sum(k * np.sign(yB3) * np.abs(yB3)**(i//3+1)*T_hT**(i%3) for i, k in enumerate(b_ikHP)) * 0.5*1025*self.ontology.L[0]**2 * self.ontology.Td[0]*Fn*Uref**2)/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)
        #print("YB is ", YB+Y_BH+Y_BP)
        #print("NB is ", NB+N_BH+N_BP)   
        #print("Uref is ", Uref)
        return XH,YH,NH,XP,XR,YR,NR,YB+Y_BH+Y_BP,NB+N_BH+N_BP,U
    def rhs(self,s, v,tend=1000): 
        XH,YH,NH,XP,XR,YR,NR,YB,NB,U=self.calculate_forces(v)
        Am=self.ontology.B[0]*self.ontology.Td[0]
        h=1.2*self.ontology.Td[0]
        beff=5*self.ontology.B[0]
        Ac=beff*h
        self.z=0.133*self.ontology.Cb[0]*(Am/(Ac-Am))**(2/3)*U**2.08
        XH=XH*self.ontology.force_switch[0]
        YH=YH*self.ontology.force_switch[0]
        NH=NH*self.ontology.force_switch[0]
        XP=XP*self.ontology.force_switch[1]
        XR=XR*self.ontology.force_switch[2]
        YR=YR*self.ontology.force_switch[2]
        NR=NR*self.ontology.force_switch[2]
        YB=YB*self.ontology.force_switch[3]
        NB=NB*self.ontology.force_switch[3]
        m26_nd=0.0108#(-4.640E-02)*(1.2)+(6.648E-02)
        m11_nd=0.0195#(-2.772E-02)*(1.2)+(5.277E-02)
        m22_nd=0.3722#(-8.757E-01)*(1.2)+(1.423E+00)
        m66_nd=0.0124#(-2.241E-02)*(1.2)+(3.929E-02)
        m11=m11_nd*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        m22=m22_nd*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        m26=m26_nd*(0.5*1025*self.ontology.L[0]**3*self.ontology.Td[0])
        m66=m66_nd*(0.5*1025*self.ontology.L[0]**4*self.ontology.Td[0])
        X_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(XP+XR)+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*XH
        Y_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YR+YB)+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*YH
        N_d=(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NR+NB)+(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*NH
        Y_m=Y_d-(self.ontology.m[0]+m11)*v[0]*v[2]
        N_m=N_d-Y_d*self.ontology.xg[0]# Y*xg already extracted
        coeff_matrix=np.array([[self.ontology.m[0]+m22, m26],[m26, self.ontology.Iz[0]+m66]])
        vdot_num_matrix=np.array([[Y_m,m26],[N_m,self.ontology.Iz[0]+m66]])
        rdot_num_matrix=np.array([[self.ontology.m[0]+m22,Y_m],[m26,N_m]])
        rates_change_modified=[(X_d+(self.ontology.m[0]+m22)*v[1]*v[2])/(self.ontology.m[0]+m11), # [not modified]
            (np.linalg.det(vdot_num_matrix))/(np.linalg.det(coeff_matrix)),#((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*(self.ontology.Iz[0]+m66)/m26-(self.ontology.m[0]+m11)*v[0]*v[2]*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*self.ontology.xg[0]-(((1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NH+NR+NB))))/((self.ontology.m[0]+m22)/m26*(self.ontology.Iz[0]+self.ontology.Jz[0])-m26), #(np.linalg.det(vdot_num_matrix))/(np.linalg.det(coeff_matrix)), # [modified]
            (np.linalg.det(rdot_num_matrix))/(np.linalg.det(coeff_matrix)),#((1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NH+NR+NB)-(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*self.ontology.xg[0]-m26*((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*(self.ontology.Iz[0]+m66)/m26-(self.ontology.m[0]+m11)*v[0]*v[2]*(self.ontology.Iz[0]+m66)/m26-(((1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NH+NR+NB))))/((self.ontology.m[0]+m22)/m26*(self.ontology.Iz[0]+m66)-m26))/(self.ontology.Iz[0]+m66), #(np.linalg.det(rdot_num_matrix))/(np.linalg.det(coeff_matrix)), # [modified]
            v[0]*np.cos(v[5])-v[1]*np.sin(v[5]), # [not modified]
            v[0]*np.sin(v[5])+v[1]*np.cos(v[5]), # [not modified]
            v[2]] # [not modified]
        return(rates_change_modified)
    def eso_rhs(self,s,v,tend=1000):
        L=3*np.ones(6)
        Gamma=[400,400,400,400,400,400,5*10**(-6)]
        XH,YH,NH,XP,XR,YR,NR,YB,NB,U=self.calculate_forces(v)
        Am=self.ontology.B[0]*self.ontology.Td[0]
        h=1.2*self.ontology.Td[0]
        beff=5*self.ontology.B[0]
        Ac=beff*h
        self.z=0.133*self.ontology.Cb[0]*(Am/(Ac-Am))**(2/3)*U**2.08
        a_1=2
        z_1=2
        a_d1=3
        z_d1=2
        x1_bar0=4
        d1_bar=0.03*x1_bar0
        a_4=2
        z_4=2
        a_d4=3
        z_d4=2
        x4_bar0=8000
        d4_bar=0.03*x4_bar0
        a_6=2
        z_6=2
        a_d6=3
        z_d6=2
        x6_bar0=1.2
        d6_bar=0.03*x6_bar0
        Y_1=a_1*x1_bar0*np.exp(-z_1*self.time)+a_d1*d1_bar*(1-np.exp(-z_d1*self.time))/z_d1
        Y_4=a_4*x4_bar0*np.exp(-z_4*self.time)+a_d4*d4_bar*(1-np.exp(-z_d4*self.time))/z_d4
        Y_6=a_6*x6_bar0*np.exp(-z_6*self.time)+a_d6*d6_bar*(1-np.exp(-z_d6*self.time))/z_d6
        #print("Threhold", Y_4)
        #print("Residual", np.abs(self.y[3]-v[3]-v[15]))
        XH=XH*self.ontology.force_switch[0]
        YH=YH*self.ontology.force_switch[0]
        NH=NH*self.ontology.force_switch[0]
        XP=XP*self.ontology.force_switch[1]
        XR=XR*self.ontology.force_switch[2]
        YR=YR*self.ontology.force_switch[2]
        NR=NR*self.ontology.force_switch[2]
        YB=YB*self.ontology.force_switch[3]
        NB=NB*self.ontology.force_switch[3]
        m26_nd=0.0108#(-4.640E-02)*(1.2)+(6.648E-02)
        m11_nd=0.0195#(-2.772E-02)*(1.2)+(5.277E-02)
        m22_nd=0.3722#(-8.757E-01)*(1.2)+(1.423E+00)
        m66_nd=0.0124#(-2.241E-02)*(1.2)+(3.929E-02)
        m11=m11_nd*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        m22=m22_nd*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        m26=m26_nd*(0.5*1025*self.ontology.L[0]**3*self.ontology.Td[0])
        m66=m66_nd*(0.5*1025*self.ontology.L[0]**4*self.ontology.Td[0])
        X_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(XP+XR)+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*XH
        Y_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YR+YB)+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*YH
        N_d=(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NR+NB)+(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*NH
        Y_m=Y_d-(self.ontology.m[0]+m11)*v[0]*v[2]
        N_m=N_d-Y_d*self.ontology.xg[0]# Y*xg already extracted
        coeff_matrix=np.array([[self.ontology.m[0]+m22, m26],[m26, self.ontology.Iz[0]+m66]])
        vdot_num_matrix=np.array([[Y_m,m26],[N_m,self.ontology.Iz[0]+m66]])
        rdot_num_matrix=np.array([[self.ontology.m[0]+m22,Y_m],[m26,N_m]])
        rates_change=[(X_d+(self.ontology.m[0]+m22)*v[1]*v[2])/(self.ontology.m[0]+m11)+L[0]*(self.y[0]-v[0]-v[12])+v[6]*Gamma[0]*(self.y[0]-v[0]-v[12])*(v[6]+1)*self.d[0]*int(self.Td[0]<=self.time)*int(np.abs(self.y[0]-v[0]-v[12])>Y_1),#((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(XH+XP+XR)+(self.ontology.m[0]+self.ontology.my[0])*self.y[1]*self.y[2])/(self.ontology.m[0]+self.ontology.mx[0])+L[0]*(self.y[0]-v[0]-v[12])+v[6]*Gamma[0]*(self.y[0]-v[0]-v[12])*(v[6]+1)*self.d[0]*int(self.Td[0]<=self.time)*int(np.abs(self.y[0]-v[0]-v[12])>Y_1), 
                (np.linalg.det(vdot_num_matrix))/(np.linalg.det(coeff_matrix))+L[1]*(self.y[1]-v[1]-v[13])+v[7]*Gamma[1]*(self.y[1]-v[1]-v[13])*(v[7]+1)*self.d[1]*int(self.Td[1]<=self.time),#((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26-(self.ontology.m[0]+self.ontology.mx[0])*self.y[0]*self.y[2]*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*self.ontology.xg[0]-(((1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NH+NR+NB))))/((self.ontology.m[0]+self.ontology.my[0])/m26*(self.ontology.Iz[0]+self.ontology.Jz[0])-m26)+L[1]*(self.y[1]-v[1]-v[13])+v[7]*Gamma[1]*(self.y[1]-v[1]-v[13])*(v[7]+1)*self.d[1]*int(self.Td[1]<=self.time),
                (np.linalg.det(rdot_num_matrix))/(np.linalg.det(coeff_matrix))+L[2]*(self.y[2]-v[2]-v[14])+v[8]*Gamma[2]*(self.y[2]-v[2]-v[14])*(v[8]+1)*self.d[2]*int(self.Td[2]<=self.time),#((1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NH+NR+NB)-(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*self.ontology.xg[0]-m26*((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YH+YR+YB)*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26-(self.ontology.m[0]+self.ontology.mx[0])*self.y[0]*v[2]*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26-(((1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NH+NR+NB))))/((self.ontology.m[0]+self.ontology.my[0])/m26*(self.ontology.Iz[0]+self.ontology.Jz[0])-m26))/(self.ontology.Iz[0]+self.ontology.Jz[0])+L[2]*(self.y[2]-v[2]-v[14])+v[8]*Gamma[2]*(self.y[2]-v[2]-v[14])*(v[8]+1)*self.d[2]*int(self.Td[2]<=self.time),
                v[0]*np.cos(v[5])-v[1]*np.sin(v[5])+L[3]*(self.ontology.yx[-1]-v[3]-v[15])+v[9]*Gamma[3]*(self.ontology.yx[-1]-v[3]-v[15])*(v[9]+1)*self.d[3]*int(np.abs(self.ontology.yx[-1]-v[3]-v[15])>Y_4),
                v[0]*np.sin(v[5])+v[1]*np.cos(v[5])+L[4]*(self.ontology.yy[-1]-v[4]-v[16])+v[10]*Gamma[4]*(self.ontology.yy[-1]-v[4]-v[16])*(v[10]+1)*self.d[4]*int(self.Td[4]<=self.time),
                v[2]+L[5]*(self.ontology.ypsi[-1]-v[5]-v[17])+v[11]*Gamma[5]*(self.ontology.ypsi[-1]-v[5]-v[17])*(v[11]+1)*self.d[-1]*int(np.abs(self.ontology.ypsi[-1]-v[5]-v[17])>Y_6),
                -L[0]*v[6]-L[0]*int(self.Td[0]<=self.time)*int(np.abs(self.y[0]-v[0]-v[12])>Y_1),
                -L[1]*v[7]-L[1]*int(self.Td[1]<=self.time),
                -L[2]*v[8]-L[2]*int(self.Td[2]<=self.time),
                -L[3]*v[9]-L[3]*int(self.Td[3]<=self.time)*int(np.abs(self.ontology.yx[-1]-v[3]-v[15])>Y_4),
                -L[4]*v[10]-L[4]*int(self.Td[4]<=self.time),
                -L[5]*v[11]-L[5]*int(self.Td[-1]<=self.time)*int(np.abs(self.ontology.ypsi[-1]-v[5]-v[17])>Y_6),
                Gamma[0]*(self.ontology.SOG[-1]*np.cos(v[5])-v[0]-v[12])*(v[6]+1)*self.d[0]*int(self.Td[0]<=self.time)*int(np.abs(self.y[0]-v[0]-v[12])>Y_1),
                Gamma[1]*(self.ontology.SOG[-1]*np.sin(v[5])-v[1]-v[13])*(v[7]+1)*self.d[1]*int(self.Td[1]<=self.time)*int(self.y[1]-v[1]-v[13]>self.measurement_noise*v[1]),
                Gamma[2]*(self.y[2]-v[2]-v[14])*(v[8]+1)*self.d[2]*int(self.Td[2]<=self.time)*int(self.y[2]-v[2]-v[14]>self.measurement_noise*v[2]),
                Gamma[3]*(self.ontology.yx[-1]-v[3]-v[15])*(v[9]+1)*self.d[3]*int(self.Td[3]<=self.time)*int(np.abs(self.y[3]-v[3]-v[15])>Y_4),
                Gamma[4]*(self.ontology.yy[-1]-v[4]-v[16])*(v[10]+1)*self.d[4]*int(self.Td[4]<=self.time)*int(self.y[4]-v[4]-v[16]>self.measurement_noise*v[4]),
                Gamma[5]*(self.ontology.ypsi[-1]-v[5]-v[17])*(v[11]+1)*self.d[-1]*int(self.Td[-1]<=self.time)*int(np.abs(self.y[-1]-v[5]-v[17])>Y_6),
                Gamma[6]*(200*self.ontology.SOG[-1])/np.sqrt((200)**2+(v[20]+200*v[18])**2)*v[20],
                self.ontology.SOG[-1]*np.sin(v[5]-self.alpha+v[18]),
                self.ontology.SOG[-1]*np.sin(v[5]-self.alpha+self.beta[-1])]
        #print(rates_change)
        return(rates_change)
    def differential_algebraic_model(self,tend):
        self.ontology.m[0]=1025*self.ontology.Cb[0]*self.ontology.L[0]*self.ontology.B[0]*self.ontology.Td[0]#/(0.5*1025*self.ontology.L[0]**3)
        self.ontology.mx[0]=0.006*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        #self.ontology.mx[0]=0.001*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        self.ontology.my[0]=0.0929*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        #self.ontology.my[0]=0.03*(0.5*1025*self.ontology.L[0]**2*self.ontology.Td[0])
        self.ontology.Iz[0]=(1/12)*self.ontology.m[0]*(self.ontology.L[0]**2+self.ontology.B[0]**2)#self.ontology.Iz[0]
        self.ontology.Jz[0]=0.0049*(0.5*1025*self.ontology.L[0]**4*self.ontology.Td[0])#/(0.5*1025*self.ontology.L[0]**5)
        #self.ontology.Jz[0]=0.0175*(0.5*1025*self.ontology.L[0]**4*self.ontology.Td[0])#/(0.5*1025*self.ontology.L[0]**5)
        res = solve_ivp(self.rhs, t_span=(0, tend), y0=[x for i,x in enumerate(self.state) if i!=5],method='RK45',rtol=1e-8,atol=1e-8)
        self.state=[res.y[ii][-1] for ii in range(len(res.y))]
        self.sensor_output() 
        self.state.insert(5,self.z)
        #print("Length of res.y is ", len(res.y))
        #res=odeint(self.rhs, x0, np.linspace(0, tend, 1000), rtol=1E-8, atol=1E-8)
        return res.t,res.y

class environment:
    def __init__(self,shapefile,width,depth):
        self.width=width #Width of the waterway
        self.depth=depth #Depth of the waterway
        self.shapefile=shapefile #Shapefile of the waterway
    def plot_environment(self):
        '''
        map = Basemap(projection='lcc', resolution='h',llcrnrlon=10.3395, llcrnrlat=59.402636, urcrnrlon=10.727223, urcrnrlat=59.507541, epsg=3857)
        map.arcgisimage(service='World_Topo_Map', xpixels=1500, verbose=True)
        # Load the Excel file
        routes_df = pd.read_excel('ASKO_Routes.xlsx', sheet_name=None)
        # Plot each route from the Excel file
        for sheet_name, df in routes_df.items():
            x,y=map(df.loc[:,'x'],df.loc[:,:].drop(columns='x'))
            plt.plot(x,y, label=sheet_name)

        # Add legend to the plot
        plt.legend()
        plt.show()
        plt.figure()
        for sheet_name, df in routes_df.items():
            plt.plot(df.loc[:,'x'],df.loc[:,:].drop(columns='x'), label=sheet_name)

        # Add legend to the plot
        plt.legend()
        plt.show()
        '''
        
        #print(self.shapefile.head())  # Display the first few rows of the data
        #print(self.shapefile.columns)  # Display the column names
        #print(self.shapefile.crs)  # Display the coordinate reference system
        #print(self.shapefile.geometry)  # Display the geometry column
        #print(self.shapefile['vrt_naam'])

        # Plot the shapefile data using matplotlib
        fig, ax = plt.subplots(figsize=(10, 10))
        #vaarwegvakken.iloc[0:1].plot(ax=ax, color='blue', edgecolor='black')

        # Convert width from meters to degrees (approximation)
        # 1 degree of latitude is approximately 111 km
        # 1 degree of longitude varies based on latitude, but we'll use an average value for simplicity
        widths_deg = [w / 111000 for w in self.width]

        # Plot each segment with the specified width
        for i, segment in enumerate(self.shapefile.iloc[0:1].geometry):
            x, y = segment.xy
            points=[[x,y] for x, y in zip(x.tolist(), y.tolist())]
            #print(points)
        '''
        outer_line1=[]
        outer_line2=[]
        for i in range(0,len(points)-1):
            outer_line1.append([points[i][0]-widths_deg[i]/2*(1/np.sqrt(np.tan((points[i][1]-points[i+1][1])/(points[i][0]-points[i+1][0]))**2+1))*np.tan((points[i][1]-points[i+1][1])/(points[i][0]-points[i+1][0])),points[i][1]+widths_deg[i]/2*(1/np.sqrt(np.tan((points[i][1]-points[i+1][1])/(points[i][0]-points[i+1][0]))**2+1))])
            outer_line2.append([points[i][0]+widths_deg[i]/2*(1/np.sqrt(np.tan((points[i][1]-points[i+1][1])/(points[i][0]-points[i+1][0]))**2+1))*np.tan((points[i][1]-points[i+1][1])/(points[i][0]-points[i+1][0])),points[i][1]-widths_deg[i]/2*(1/np.sqrt(np.tan((points[i][1]-points[i+1][1])/(points[i][0]-points[i+1][0]))**2+1))])
        outer_line1.append([points[len(points)-1][0]-widths_deg[len(points)-1]/2*(1/np.sqrt(np.tan((points[len(points)-1][1]-points[len(points)-2][1])/(points[len(points)-1][0]-points[len(points)-2][0]))**2+1))*np.tan((points[len(points)-1][1]-points[len(points)-2][1])/(points[len(points)-1][0]-points[len(points)-2][0])),points[len(points)-1][1]+widths_deg[len(points)-1]/2*(1/np.sqrt(np.tan((points[len(points)-1][1]-points[len(points)-2][1])/(points[len(points)-1][0]-points[len(points)-2][0]))**2+1))])
        outer_line2.append([points[len(points)-1][0]+widths_deg[len(points)-1]/2*(1/np.sqrt(np.tan((points[len(points)-1][1]-points[len(points)-2][1])/(points[len(points)-1][0]-points[len(points)-2][0]))**2+1))*np.tan((points[len(points)-1][1]-points[len(points)-2][1])/(points[len(points)-1][0]-points[len(points)-2][0])),points[len(points)-1][1]-widths_deg[len(points)-1]/2*(1/np.sqrt(np.tan((points[len(points)-1][1]-points[len(points)-2][1])/(points[len(points)-1][0]-points[len(points)-2][0]))**2+1))])
        outer_line1_x=[outer_line1[i][0] for i in range(0,len(outer_line1))]
        outer_line1_x.pop(0)
        outer_line1_x.pop(0)
        outer_line1_y=[outer_line1[i][1] for i in range(0,len(outer_line1))]
        outer_line1_y.pop(0)
        outer_line1_y.pop(0)
        outer_line2_x=[outer_line2[i][0] for i in range(0,len(outer_line2))]
        outer_line2_x.pop(0)
        outer_line2_x.pop(0)
        outer_line2_y=[outer_line2[i][1] for i in range(0,len(outer_line2))]
        outer_line2_y.pop(0)
        outer_line2_y.pop(0)
        x=np.delete(x,[0,1])
        y=np.delete(y,[0,1])
        '''
        ax.plot(x,y, linewidth=1, color='red',linestyle='--')
        '''
        ax.plot(outer_line1_x,outer_line1_y, linewidth=1, color='orange')
        ax.plot(outer_line2_x,outer_line2_y, linewidth=1, color='orange')
        ax.fill_between(outer_line2_x, outer_line2_y, ax.get_ylim()[0], color='orange')
        ax.fill_between(outer_line2_x, outer_line2_y, outer_line1_y, color='b')
        ax.fill_between(outer_line1_x, ax.get_ylim()[1], outer_line1_y,  color='orange')
        '''
        ax.set_title('Zandkreek en Veerse Meer')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.show()  