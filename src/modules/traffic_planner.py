import do_mpc as mpc
import casadi
import sys
import numpy as np
sys.path.insert(1, 'src/')

from channel import *
from intel_guidance import *

def calculate_forces(self,v,delta,Np,depth,sinkage,yB3_k):
    # input: self - constants for the vessel; v - vessel states;
    # sequence of v: [u,v,r,x,y,\psi] + [z]
    # input: delta - rudder angle, Np - propeller revolutional speed
    # input: water depth; sinkage, yB3_k - for bank effects
    # Note that delta in rads
    #================================================
    # 1. Constants
    #================================================
    R0=0.0428                    #[checked]
    t=0.326                      #[checked]
    wp0=0.576                    #[checked]
    p=0.328
    C1=0.98
    Dp=1.8                       #[checked]
    lp=(self.ontology.L[0]/2+self.ontology.xg[0])/(self.ontology.L[0])
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
    # yp=self.distance_from_bank1
    # ys=self.distance_from_bank2
    # yp=-(0.5*wb+v[4])
    # ys=0.5*wb-v[4]
    # yB3 = 0.5*self.ontology.B[0]*(1/yp+1/ys)  # Non-dimensional ship-bank distance
    yB3=yB3_k # use the current yB3 for calculation
    # Td_affected=self.ontology.Td[0]+v[6] # for bank effect modification if necessary
    a_ikH=[-0.461+0.368/self.ontology.Cb[0],-0.173+0.0114*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0],0.00634-0.00046*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0], 1.36-0.0658*self.ontology.L[0]/self.ontology.Td[0],0.257-0.0896*self.ontology.B[0]/self.ontology.Td[0],-0.0191+0.00634*self.ontology.B[0]/self.ontology.Td[0]]
    b_ikH=[-0.119+2.11*self.ontology.Td[0]/self.ontology.L[0],-0.0354+0.195*self.ontology.B[0]/self.ontology.L[0],2.29*10**(-4)-6.92*10**(-5)*self.ontology.B[0]/self.ontology.Td[0], 0.134-2.35*self.ontology.Td[0]/self.ontology.L[0],-4.23*10**(-2)+2.93*10**(-3)*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0],1.23*10**(-3)-8.53*10**(-5)*self.ontology.Cb[0]*self.ontology.L[0]/self.ontology.Td[0]]
    a_iKP=[0.1 - 0.0524 * self.ontology.Td[0] / self.ontology.B[0], -0.0087 - 0.0386 * self.ontology.B[0] / self.ontology.L[0], -0.000105 - 0.000000923 * self.ontology.L[0] / self.ontology.B[0], 0.0592 - 0.0211 * self.ontology.B[0] / self.ontology.Td[0], -0.000282 - 0.000242 * self.ontology.L[0] / self.ontology.B[0], 0.000232 - 0.0000198 * self.ontology.L[0] / self.ontology.B[0]]
    b_iKP=[-0.00495 + 0.125 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.L[0], 0.00129 + 0.00537 * self.ontology.Td[0] / self.ontology.B[0], -0.000268 + 0.000878 * self.ontology.Td[0] / self.ontology.B[0],0.00381 - 0.0129 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.B[0], 0.00344 - 0.0011 * self.ontology.B[0]/ self.ontology.Td[0], 0.000488 - 0.00152 * self.ontology.Td[0] / self.ontology.B[0]]
    a_ikHP=[-0.237 + 0.422 * self.ontology.Td[0] / self.ontology.B[0], 0.0289 - 0.0022 * self.ontology.L[0] / self.ontology.B[0], -0.00657 + 0.00118 * self.ontology.L[0] / self.ontology.B[0], 0.419 - 0.0479 * self.ontology.L[0] / self.ontology.B[0], -0.0449 + 0.00998 * self.ontology.L[0] / self.ontology.B[0], 0.0106 + 0.0424 * self.ontology.Td[0] / self.ontology.B[0]]
    b_ikHP=[-0.289 + 5.71 * self.ontology.Td[0] / self.ontology.L[0], -0.122 + 0.353 * self.ontology.Td[0] / self.ontology.B[0], 0.00787 - 0.0233 * self.ontology.Td[0] / self.ontology.B[0],-0.0933 + 0.0157 * self.ontology.L[0] / self.ontology.B[0], 0.26 - 0.795 * self.ontology.Td[0] / self.ontology.B[0], -0.0199 + 0.0598 * self.ontology.Td[0] / self.ontology.B[0]]
    #================================================
    # 2. Problem defined variables
    #================================================
    U=casadi.sqrt(casadi.power((v[0]-0.02),2)+casadi.power((v[1]-0.01),2))
    s=1-U/(Np*Dp)                #[checked]
    #print("Np is ", Np)
    #================================================
    # 3. Hull forces calculation
    #================================================
    Fn=v[0]/casadi.sqrt(9.81*self.ontology.L[0])
    bm=-casadi.arctan((v[1]-self.ontology.xg[0]*v[2])/v[0])
    r_nd=v[2]*self.ontology.L[0]/U # Non-dimensional yaw rate
    try:
        XH=-R0*casadi.power(casadi.cos(bm),2)+self.ontology.Xbb[0]*casadi.power(bm,2)+self.ontology.Xbr[0]*bm*r_nd+self.ontology.Xrr[0]*casadi.power(r_nd,2)
        YH=self.ontology.Yb[0]*bm+self.ontology.Yr*r_nd+self.ontology.Ybbb[0]*casadi.power(bm,3)+self.ontology.Ybbr[0]*casadi.power(bm,2)*r_nd+self.ontology.Ybrr[0]*bm*casadi.power(r_nd,2)+self.ontology.Yrrr[0]*casadi.power(r_nd,3)
        NH=self.ontology.Nb[0]*bm+self.ontology.Nr*r_nd+self.ontology.Nbbb[0]*casadi.power(bm,3)+self.ontology.Nbbr[0]*casadi.power(bm,2)*r_nd+self.ontology.Nbrr[0]*bm*casadi.power(r_nd,2)+self.ontology.Nrrr[0]*casadi.power(r_nd,3)
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
    b=-casadi.arctan((v[1])/v[0])
    bp=b-lp*r_nd
    wp=wp0*casadi.exp(-C1*casadi.power(bp,2))
    Jp=v[0]*(1-wp)/(Np*Dp)
    KT=-0.326*p*Jp-0.2005*Jp+0.5234*p-0.0398
    XP=(2*(1-t)*1025*casadi.power(Np,2)*casadi.power(Dp,4)*KT)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    #================================================
    # 4. Rudder forces calculation
    #================================================
    delta_rad = delta  # Convert to radians
    uP=v[0]*(1-s)
    fa = 6.13/(1+2.25)  # Gradient of the normal force coefficient with respect to rudder angle
    bR=b-lR*r_nd
    vR=v[1]*gR*bR
    eta=Dp/(2)
    uR=e*uP/(1-s)*casadi.sqrt(1-2*(1-eta*k)*s+(1-eta*k*(2-k))*casadi.power(s,2))
    aR=delta_rad-casadi.arctan(vR/uR)
    UR=casadi.sqrt(casadi.power(uR,2)+casadi.power(vR,2))
    FN=(0.5*1025*AR*casadi.power(UR,2)*fa*casadi.sin(aR))
    XR=-(1-tR)*2*FN*casadi.sin(delta_rad)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    YR=-(1+aH)*2*FN*casadi.cos(delta_rad)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    NR=-(xR+aH*xH)*2*FN*casadi.cos(delta_rad)/(1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U,2))
    #================================================
    # 5. Bank effects calculation
    #================================================
    Uref=casadi.sqrt(casadi.fabs((1025*0.25*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))*XP/(1025*casadi.pi*casadi.power(Dp,2/8)))) # Reference velocity
    # h_eff=depth-v[6]
    h_eff=depth-sinkage
    T_hT=self.ontology.Td[0]/(h_eff-self.ontology.Td[0])
    # Y_BH = sum(a * casadi.power(yB3,(i%3))*casadi.power(T_hT,(i%3))  for i, a in enumerate(a_ikH)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0] * (casadi.power(v[0],2))/(1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U,2))
    # N_BH = sum(b * casadi.power(yB3,(i%3))*casadi.power(T_hT,(i%3))  for i, b in enumerate(b_ikH)) * 0.5*1025*casadi.power(self.ontology.L[0],2)* self.ontology.Td[0] * (casadi.power(v[0],2))/(1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U,2))
    # Y_BP = sum(p * casadi.power(yB3,(i%3))*casadi.power(T_hT,(i%3))  for i, p in enumerate(a_iKP)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0]*(casadi.power(Uref,2))/(1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U,2))
    # N_BP = sum(q * casadi.power(yB3,(i%3))*casadi.power(T_hT,(i%3))  for i, q in enumerate(b_iKP)) * 0.5*1025*casadi.power(self.ontology.L[0],2)* self.ontology.Td[0]*(casadi.power(Uref,2))/(1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U,2))
    # YB = (sum(hp * casadi.power(yB3,(i%3))*casadi.power(T_hT,(i%3))  for i, hp in enumerate(a_ikHP)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0]*Fn)/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    # NB = (sum(k * casadi.power(yB3,(i%3))*casadi.power(T_hT,(i%3)) for i, k in enumerate(b_ikHP)) * 0.5*1025*casadi.power(self.ontology.L[0],2) * self.ontology.Td[0]*Fn*casadi.power(Uref,2))/(1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U,2))
    Y_BH = sum(a * casadi.sign(yB3)*casadi.power(casadi.fabs(yB3),(i//3+1))*casadi.power(T_hT,(i%3))  for i, a in enumerate(a_ikH)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0] * (casadi.power(v[0],2))/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    N_BH = sum(b * casadi.sign(yB3)*casadi.power(casadi.fabs(yB3),(i//3+1))*casadi.power(T_hT,(i%3))  for i, b in enumerate(b_ikH)) * 0.5*1025*self.ontology.L[0]**2 * self.ontology.Td[0] * (casadi.power(v[0],2))/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*casadi.power(U,2))
    Y_BP = sum(p * casadi.sign(yB3)*casadi.power(casadi.fabs(yB3),(i//3+1))*casadi.power(T_hT,(i%3))  for i, p in enumerate(a_iKP)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0]*(casadi.power(Uref,2))/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    N_BP = sum(q * casadi.sign(yB3)*casadi.power(casadi.fabs(yB3),(i//3+1))*casadi.power(T_hT,(i%3))  for i, q in enumerate(b_iKP)) * 0.5*1025*self.ontology.L[0]**2 * self.ontology.Td[0]*(casadi.power(Uref,2))/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*casadi.power(U,2))
    YB = (sum(hp * casadi.sign(yB3)*casadi.power(casadi.fabs(yB3),(i//3+1))*casadi.power(T_hT,(i%3))  for i, hp in enumerate(a_ikHP)) * 0.5*1025*self.ontology.L[0] * self.ontology.Td[0]*Fn*casadi.power(Uref,2))/(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U,2))
    NB = (sum(k * casadi.sign(yB3)*casadi.power(casadi.fabs(yB3),(i//3+1))*casadi.power(T_hT,(i%3)) for i, k in enumerate(b_ikHP)) * 0.5*1025*self.ontology.L[0]**2 * self.ontology.Td[0]*Fn*casadi.power(Uref,2))/(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*casadi.power(U,2))
    return XH,YH,NH,XP,XR,YR,NR,YB+Y_BH+Y_BP,NB+N_BH+N_BP,U

def MPC_controller(myvessel,psi_d):
    MPC_prediction_horizon=5
    MPC_time_step=0.5
    model_type = 'continuous'
    model = mpc.model.Model(model_type) 
    # Set state variables
    x_1=model.set_variable(var_type='_x',var_name='x_1',shape=(1,1))  # X-position
    x_2=model.set_variable(var_type='_x',var_name='x_2',shape=(1,1))  # Y-position
    x_3=model.set_variable(var_type='_z',var_name='x_3',shape=(1,1))  # Z-position
    x_4=model.set_variable(var_type='_x',var_name='x_4',shape=(1,1))  # Yaw angle
    x_5=model.set_variable(var_type='_x',var_name='x_5',shape=(1,1))  # State derivatives [surge speed, sway speed, yaw rate]
    x_6=model.set_variable(var_type='_x',var_name='x_6',shape=(1,1))
    x_7=model.set_variable(var_type='_x',var_name='x_7',shape=(1,1))
    # Set control variables
    Np=model.set_variable(var_type='_u',var_name='Np',shape=(1,1)) # Desired propeller speed
    delta=model.set_variable(var_type='_u',var_name='delta',shape=(1,1)) # Desired rudder angle
    #p=model.set_variable(var_type='_u',var_name='p',shape=(1,1)) # Desired propeller pitch
    # Set parameters
    psi_ref=model.set_variable(var_type='_tvp',var_name='psi_ref',shape=(1,1)) # Desired heading reference
    # Set model equations
    model.set_rhs('x_1',x_5*casadi.cos(x_4)-x_6*casadi.sin(x_4))
    model.set_rhs('x_2',x_5*casadi.sin(x_4)+x_6*casadi.cos(x_4))
    v_sym = casadi.vertcat(x_5, x_6, x_7, x_1, x_2, x_4, x_3)
    self=myvessel
    # check the minimum depth within the horizon assuming current speed
    u_now_x=self.y[0]*np.cos(self.y[-1])-self.y[1]*np.sin(self.y[-1]) # ground speed in x-direction
    v_now_y=self.y[0]*np.sin(self.y[-1])+self.y[1]*np.cos(self.y[-1]) # ground speed in y-direction
    depths=[]
    for i in range(MPC_prediction_horizon+1):
        depths.append(cal_depth([self.y[3]+i*MPC_time_step*u_now_x,self.y[4]+i*MPC_time_step*v_now_y],self.depths_data))
    # depths[0] is the current depth, np.min(depths) is the minimum depth
    x_grid_horizon=[self.y[3]+i*MPC_time_step*u_now_x for i in range (MPC_prediction_horizon+1)] # the available x coordinates
    depth_inter= casadi.interpolant("D_INT","bspline",[x_grid_horizon],depths) # interpolant for the depths
    Am=self.ontology.B[0]*self.ontology.Td[0]
    beff=4*self.ontology.B[0] # consider W=4B constant
    Ac=beff*depths[0]
    S_f=0.98*Am/Ac
    K_barrass=5.74*S_f**0.76
    if K_barrass<1:
        K_barrass=1
    if K_barrass>2:
        K_barrass=2
    z0_calc=0.01*K_barrass*self.ontology.Cb[0]*((self.y[0]**2+self.y[1]**2))*(3.6/1.852)**2
    # print(f"{z0_calc:.4f} vs. {self.z:.4f}") # error in z initial prediction
    # XH,YH,NH,XP,XR,YR,NR,YB,NB,U=calculate_forces(self,v_sym,delta,Np,depth=depths[0],sinkage=z0_calc)
    yB3_current=0.5*self.ontology.B[0]*(1/(-(0.5*beff+myvessel.y[4]))+1/(0.5*beff-myvessel.y[4]))
    XH,YH,NH,XP,XR,YR,NR,YB,NB,U=calculate_forces(self,v_sym,delta,Np,depth=depths[0],sinkage=x_3,yB3_k=yB3_current)
    # h=1.2*self.ontology.Td[0] # consider h=1.2Td constant
    h=np.min(depths)# from the minimum depth get minimum Ac to get maximum K_barrrass
    # h=depth_inter(x_1)
    Ac=beff*h 
    S_f=0.98*Am/Ac
    K_barrass=5.74*S_f**0.76
    # K_barrass=casadi.fmax(K_barrass,1)
    # K_barrass=casadi.fmin(K_barrass,2)
    if K_barrass<1:
        K_barrass=1
    if K_barrass>2:
        K_barrass=2
    # U_safe = U
    # alg_fun = x_3 - 0.133 * self.ontology.Cb[0] * casadi.power((Am / (Ac - Am)), 2 / 3) * casadi.power(U_safe, 2.08)
    alg_fun = x_3 - 0.01*K_barrass*self.ontology.Cb[0]*(U*3.6/1.852)**2 # assume K_barrass as the maximum within the horizon
    model.set_alg('x_3',alg_fun)
    model.set_rhs('x_4',x_7)
    # m26=0.0006*(0.5*1025*casadi.power(self.ontology.L[0],3)*self.ontology.Td[0])
    XH=XH*self.ontology.force_switch[0]
    YH=YH*self.ontology.force_switch[0]
    NH=NH*self.ontology.force_switch[0]
    XP=XP*self.ontology.force_switch[1]
    XR=XR*self.ontology.force_switch[2]
    YR=YR*self.ontology.force_switch[2]
    NR=NR*self.ontology.force_switch[2]
    YB=YB*self.ontology.force_switch[3]
    NB=NB*self.ontology.force_switch[3]
    m11_nd, m22_nd, m26_nd, m66_nd = cal_addedmass_nd(hdratio=1.2)
    Td_affected=self.ontology.Td[0]+0.01*K_barrass*self.ontology.Cb[0]*(U*3.6/1.852)**2  # assume K_barrass as the maximum within the horizon
    m11=m11_nd*(0.5*1025*self.ontology.L[0]**2*Td_affected)
    m22=m22_nd*(0.5*1025*self.ontology.L[0]**2*Td_affected)
    m26=m26_nd*(0.5*1025*self.ontology.L[0]**3*Td_affected)
    m66=m66_nd*(0.5*1025*self.ontology.L[0]**4*Td_affected)
    # X_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(XP+XR)+(1025*0.5*self.ontology.L[0]*Td_affected*U**2)*XH
    # Y_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*YR+(1025*0.5*self.ontology.L[0]*Td_affected*U**2)*(YH+YB)
    # N_d=(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*NR+(1025*0.5*self.ontology.L[0]**2*Td_affected*U**2)*(NH+NB)
    X_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(XP+XR)+(1025*0.5*self.ontology.L[0]*Td_affected*U**2)*XH
    Y_d=(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*U**2)*(YR+YB)+(1025*0.5*self.ontology.L[0]*Td_affected*U**2)*YH
    N_d=(1025*0.5*self.ontology.L[0]**2*self.ontology.Td[0]*U**2)*(NR+NB)+(1025*0.5*self.ontology.L[0]**2*Td_affected*U**2)*NH
    Y_m=Y_d-(self.ontology.m[0]+m11)*x_5*x_7
    N=N_d-Y_d*self.ontology.xg[0] # Y*xg already extracted
    vdot_num_matrix_det=Y_m*(self.ontology.Iz[0]+m66)-N*m26 # np.array([[Y_m,m26],[N,self.ontology.Iz[0]+m66]])
    rdot_num_matrix_det=(self.ontology.m[0]+m22)*N-m26*Y_m # np.array([[self.ontology.m[0]+m22,Y_m],[m26,N]])
    coeff_matrix_det=(self.ontology.m[0]+m22)*(self.ontology.Iz[0]+m66)-m26*m26 # np.array([[self.ontology.m[0]+m22, m26],[m26, self.ontology.Iz[0]+m66]])
    model.set_rhs('x_5',(X_d+(self.ontology.m[0]+m22)*x_6*x_7)/(self.ontology.m[0]+m11))
    model.set_rhs('x_6',vdot_num_matrix_det/coeff_matrix_det)
    model.set_rhs('x_7',rdot_num_matrix_det/coeff_matrix_det)
    # model.set_rhs('x_5',((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U_safe,2))*(XH+XP+XR)+(self.ontology.m[0]+self.ontology.my[0])*x_6*x_7)/(self.ontology.m[0]+self.ontology.mx[0]))
    # model.set_rhs('x_6',((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U_safe,2))*(YH+YR+YB)*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26-(self.ontology.m[0]+self.ontology.mx[0])*x_5*x_7*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26+(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U_safe,2))*(YH+YR+YB)*self.ontology.xg[0]-(((1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U_safe,2))*(NH+NR+NB))))/((self.ontology.m[0]+self.ontology.my[0])/m26*(self.ontology.Iz[0]+self.ontology.Jz[0])-m26))
    # model.set_rhs('x_7',((1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U_safe,2))*(NH+NR+NB)-(1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U_safe,2))*(YH+YR+YB)*self.ontology.xg[0]-m26*((1025*0.5*self.ontology.L[0]*self.ontology.Td[0]*casadi.power(U_safe,2))*(YH+YR+YB)*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26-(self.ontology.m[0]+self.ontology.mx[0])*x_5*x_7*(self.ontology.Iz[0]+self.ontology.Jz[0])/m26-(((1025*0.5*casadi.power(self.ontology.L[0],2)*self.ontology.Td[0]*casadi.power(U_safe,2))*(NH+NR+NB))))/((self.ontology.m[0]+self.ontology.my[0])/m26*(self.ontology.Iz[0]+self.ontology.Jz[0])-m26))/(self.ontology.Iz[0]+self.ontology.Jz[0]))
    #p_template = vesselmpc.get_p_template(11)
    model.setup()
    # Create controller
    vesselmpc = mpc.controller.MPC(model)
    # Set MPC parameters
    vesselmpc.set_param(
        n_horizon= MPC_prediction_horizon,
        t_step= MPC_time_step,
        store_full_solution= False,
        nlpsol_opts={"ipopt.linear_solver": "mumps","ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0}#{'ipopt.linear_solver': 'mumps'},
        # nlpsol_opts={"ipopt.linear_solver": "mumps", "ipopt.sb": "yes"}#{'ipopt.linear_solver': 'mumps'},
    )
    #p_template = vesselmpc.get_p_template(n_combinations=1)
    #def p_fun(t_now):
    #    p_template[0]['psi_ref']=psi_d
    #    return p_template
    #vesselmpc.set_p_fun(p_fun)
    tvp_template = vesselmpc.get_tvp_template()
    def tvp_fun(t_now):
        for k in range(vesselmpc.settings.n_horizon+1):
            tvp_template['_tvp',k,'psi_ref']= psi_d
        # print(psi_d)
        return tvp_template
    vesselmpc.set_tvp_fun(tvp_fun)
    # Objective function, ship speed included
    mterm = (1)*casadi.power((-x_4+psi_ref),2) + (-3e-6)*(casadi.power(x_5,2)+casadi.power(x_6,2))
    lterm = (1)*casadi.power((-x_4+psi_ref),2) + (-3e-6)*(casadi.power(x_5,2)+casadi.power(x_6,2))
    vesselmpc.set_objective(mterm=mterm, lterm=lterm)
    vesselmpc.set_rterm( # penalty on control input change rate
        Np=1e-7,
        delta=1e-7
    )
    # Constraints
    #vesselmpc.bounds['lower','_u','p'] = 0.328
    #vesselmpc.bounds['upper','_u','p'] = 0.328
    vesselmpc.bounds['lower','_u','Np'] = 100/60
    vesselmpc.bounds['upper','_u','Np'] = 500/60
    vesselmpc.bounds['lower','_u','delta'] = -35*np.pi/180
    vesselmpc.bounds['upper','_u','delta'] = 35*np.pi/180
    # vesselmpc.bounds['lower','_z','x_3'] = 0
    # vesselmpc.bounds['upper','_z','x_3'] = self.ontology.Td[0]*0.2
    # nonlinear_expr= casadi.fabs(-casadi.atan2(x_6,x_5))*180/np.pi
    # vesselmpc.set_nl_cons('cons', nonlinear_expr, 0.2)
    U_max=7*1.852/3.60 # maximum speed as 7 knots
    nonlinear_expr = x_5**2 + x_6**2
    vesselmpc.set_nl_cons('cons', nonlinear_expr, U_max**2)
    # == grounding constraint: x_3 + Td + UKC_min - h(SB) <= 0
    UKC_min = 0.05  # hard constraint for UKC
    UKC_buff = 0.15  # reserved UKC value for safety
    # vesselmpc.bounds['upper','_z','x_3'] = np.min(depths) - self.ontology.Td[0]  # grounding constraint as ub
    nonlinear_expr = x_3 + self.ontology.Td[0] + UKC_min + UKC_buff - np.min(depths)
    # nonlinear_expr = x_3 + self.ontology.Td[0] + UKC_min + UKC_buff - depth_inter(x_1)
    # nonlinear_expr = x_3 + self.ontology.Td[0] - np.min(depths)
    # print(nonlinear_expr)
    # nonlinear_ub = UKC_min + cal_depth([self.y[0],self.y[1]]) - self.ontology.Td[0] # get depth from current coordinates
    # nonlinear_cons=vesselmpc.set_nl_cons('cons_g', nonlinear_expr, 0) # grounding constraint, note might be violated
    nonlinear_cons=vesselmpc.set_nl_cons('cons_g', nonlinear_expr, 0, soft_constraint=True, penalty_term_cons=1e-3, maximum_violation=UKC_buff) # grounding constraint (soft)
    # print(f"({self.y[3]:.2f},{self.y[4]:.3f}) "+f"{nonlinear_cons}")
    vesselmpc.setup()
    # print(np.array(self.y[3:5]+self.y[5:]+self.y[0:3]).reshape(-1,1))
    vesselmpc.x0 = np.array(self.y[3:5]+self.y[5:]+self.y[0:3]).reshape(-1,1) # x,y,\psi,u,v,r
    # vesselmpc.z0 = np.array(self.z) # initial value of z, but self.z cannot be used here since it is not output
    vesselmpc.z0=z0_calc # initial z based on the calcualtion of sensor output
    vesselmpc.u0 = np.array([self.Np[-1],self.u[-1]*np.pi/180]) # initial guess as the current control, note that Np>=100
    vesselmpc.set_initial_guess()
    #self.psi_ref = psi_d
    # Set initial state
    #vesselmpc.set_initial_guess()
    #estimator = mpc.estimator.StateFeedback(model)
    #simulator = mpc.simulator.Simulator(model)
    #model.check_consistency()

    # params_simulator = {
    #     'integration_tool': 'idas',
    #     'abstol': 1e-8,
    #     'reltol': 1e-8,
    #     't_step': 0.04
    # }
    # p_num = simulator.get_p_template()
    # p_num['psi_ref'] = psi_d
    # p_num['XH'] = XH1*self.ontology.force_switch[0]
    # p_num['YH'] = YH1*self.ontology.force_switch[0]
    # p_num['NH'] = NH1*self.ontology.force_switch[0]
    # p_num['XP'] = XP1*self.ontology.force_switch[1]
    # p_num['XR'] = XR1*self.ontology.force_switch[2]
    # p_num['YR'] = YR1*self.ontology.force_switch[2]
    # p_num['NR'] = NR1*self.ontology.force_switch[2]
    # p_num['YB'] = YB1*self.ontology.force_switch[3]
    # p_num['NB'] = NB1*self.ontology.force_switch[3]
    # p_num['U'] = U1
    # def p_fun(t_now):
    #     return p_num
    # simulator.set_p_fun(p_fun)
    # simulator.set_param(**params_simulator)
    # simulator.setup()
    # x0=np.array(self.y[3:5]+self.y[6:]+self.y[0:3]).reshape(-1,1)
    # z0=np.array([self.y[5]]).reshape(-1,1)
    # vesselmpc.x0 = x0
    # vesselmpc.z0=z0
    # simulator.x0 = x0
    # simulator.z0=z0
    # estimator.x0 = x0
    # estimator.z0 = z0
    # simulator.reset_history()
    # simulator.set_initial_guess()
    #print("x0",x0)
    #print(dx_next.shape)
    #print("z0",z0)
    #print("PSI_D",psi_d)
    #print("sim_x_num =", simulator.x0)
    #print("sim_z_num =", simulator.z0)
    return vesselmpc #, simulator, estimator

def get_next_mpc_step(vessel):
    x0=np.array(vessel.state[3:5]+vessel.state[6:]+vessel.state[0:3]).reshape(-1,1)
    u = vessel.MPC.make_step(x0)
    # y_next=vessel.simulator.make_step(u)
    # x=vessel.estimator.make_step(y_next)
    return u

def PID_controller(self, psi_d, dt):
    """
    Implements a PID (Proportional-Integral-Derivative) controller for a vessel.
    Parameters:
        vessel (object): The vessel object containing its state, route, and other attributes.
            - vessel.state (list): The current state of the vessel, where:
                - vessel.state[-1] represents the current heading angle (psi).
                - vessel.state[2] represents the current yaw rate (r).
            - vessel.route (list): The predefined route for the vessel.
            - vessel.chi (list): A list storing the desired heading angles (psi_d).
            - vessel.aggregated_error (float): The accumulated heading error for the integral term.
            - vessel.error (float): The current heading error.
        dt (float): The time step for the controller.
        *args: Additional arguments (not used in the current implementation).
    Returns:
        float: The rudder angle (delta) in degrees.
    Notes:
        - The controller calculates the desired heading angle (psi_d) using a Line-of-Sight (LOS) guidance method.
        - The PID control law is applied to compute the rudder angle (delta) based on the heading error, 
          the integral of the error, and the derivative of the error.
        - The proportional gain (KP), integral time (Ti), and derivative time (Td) are fixed constants.
        - The rudder angle is constrained to a maximum of ±35 degrees to ensure realistic actuation limits.
    """
    #K_p=10
    #T_i=3
    #T_d=1
    #P = -KP * error
    #I = -KP / Ti * vessel.aggregated_error * dt
    #D = -KP * Td * (r-r_d) / dt
    #delta = P + I + D
    A=4
    Yd=-3*A/self.ontology.L[0]**2
    Nd=-Yd/2
    xg_nd=self.ontology.xg[0]/self.ontology.L[0]
    Iz_nd=self.ontology.Iz[0]/(0.5*1025*self.ontology.L[0]**5)
    m_nd=self.ontology.m[0]/(0.5*1025*self.ontology.L[0]**3)
    Yvdot=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1+0.16*self.ontology.Cb[0]*self.ontology.B[0]/self.ontology.Td[0]-5.1*(self.ontology.B[0]/self.ontology.L[0])**2)
    Yrdot=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(0.67*(self.ontology.B[0]/self.ontology.L[0])-0.0033*(self.ontology.B[0]/self.ontology.Td[0])**2)
    Nvdot=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1.1*(self.ontology.B[0]/self.ontology.L[0])-0.041*(self.ontology.B[0]/self.ontology.Td[0]))
    Nrdot=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1/12+0.017*self.ontology.Cb[0]*(self.ontology.B[0]/self.ontology.Td[0])-0.33*(self.ontology.B[0]/self.ontology.L[0]))
    #if (self.y[3]*0.2<=600):
    Yv=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1+0.4*self.ontology.Cb[0]*(self.ontology.B[0]/self.ontology.Td[0]))
    Nv=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(0.5+2.4*self.ontology.Td[0]/self.ontology.L[0])
    Nr=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(1/4+0.039*(self.ontology.B[0]/self.ontology.Td[0])-0.56*(self.ontology.B[0]/self.ontology.L[0]))
    Yr=-np.pi*(self.ontology.Td[0]/self.ontology.L[0])**2*(-1/2+2.2*(self.ontology.B[0]/self.ontology.L[0])-0.08*(self.ontology.B[0]/self.ontology.Td[0]))
    # else:
    #     k=2*self.ontology.Td[0]/self.ontology.L[0]
    #     h_d=1.2
    #     ke=k/(0.5/h_d+(np.pi/(2*h_d)/ np.tan(np.pi/(2*h_d))))
    #     Yv=-np.pi/2*ke-1.4*self.ontology.Cb[0]*(self.ontology.B[0]/self.ontology.L[0])
    #     Yr=np.pi/4*ke
    #     Nv=-ke
    #     Nr=-0.54*ke+ke**2
    K_nd=(Nv*Yd-Yv*Nd)/(Nr*Yv+(-Yr+m_nd)*Nv)
    T1_2=((Yvdot-m_nd)*(Nr-m_nd*xg_nd)+(Nrdot-Iz_nd)*Yv-(Yrdot-m_nd*xg_nd)*Nv-(Nvdot-m_nd*xg_nd)*(Yr-m_nd))/(Yv*(Nr-m_nd*xg_nd)-Nv*(Yr-m_nd))
    T3=((Nv-m_nd*xg_nd)*Yd-(Yvdot-m_nd)*Nd)/(Nv*Yd-Yv*Nd)
    T_nd=T1_2-T3
    U=2.5
    T = T_nd*self.ontology.L[0]/U #27.0;                       # Nomoto gains at U = 9 m/s
    K = K_nd*U/self.ontology.L[0]#0.18;
    # print(self.type)
    # print(T)
    # print(K)
    # print('wn min', 1/(T*np.sqrt(-2*0.9**2+1+np.sqrt(4*0.9**4-4*0.9**2+2))))
    wn = 0.1;                       # Closed-loop natural frequency (rad/s)
    zeta=0.95;
    Kp =(T/K) * wn**2    
    Kd = (T/K) * (2 * zeta * wn - 1/T)
    Td =Kd / Kp;                
    Ti =10 / wn;
    #Td=1
    #print("Kp is ", Kp)
    #print("Ti is ", Ti)
    # State variables
    psi = self.ontology.ypsi[-1]  # Current heading % @Nikos took this as vessel.arctan2. Why?? 
    #print("Heading is ",psi)
    # print("Vessel ", self.type)
    # print("=================== ")
    # print("psi is ", psi)
    # print("psi_d is ", psi_d)
    r = self.state[2]*(1.0+self.measurement_noise*np.random.uniform(-1,1,size=None))    # Current yaw rate
    error_psi = wrap_angle_to_pmpi(psi_d - psi)
    # print ("Controller error ", error_psi)
    #self.sum_error = error_psi+ dt* self.error_old
    r_d = wrap_angle_to_pmpi(psi_d - self.psi_d_prev) /dt
    self.aggregated_error=self.aggregated_error+error_psi*dt
    # PID control law for rudder angle
    delta = Kp * (error_psi + (1 / Ti) * self.aggregated_error + Td * (-r + r_d))
    delta = np.clip(delta, -35*np.pi/180, 35*np.pi/180)
    # Update old values
    #self.psi_d_prev = psi_d
    #self.chi.append(psi_d)
    #self.error_old = error_psi
    # Store control action
    #self.control = {'delta': delta}
    return delta  # Commanded rudder angle

def plan_route(start,stop):
    """
    Plans a route between a start and stop point by generating reference points.
    Parameters:
        start (tuple): A tuple (x, y) representing the starting coordinates.
        stop (tuple): A tuple (x, y) representing the stopping coordinates.
    Returns:
        tuple: Two numpy arrays (x_ref, y_ref) representing the x and y coordinates 
               of the reference points along the route.
    """
    x_ref=np.linspace(start[0],stop[0],20)
    y_ref=np.linspace(start[1],stop[1],20)
    wp_pos = np.array([x_ref,y_ref])
    return wp_pos

def wrap_angle_to_pmpi(angle):
    """
    Wraps an angle to the range [-pi, pi].
    """
    wrapped_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return wrapped_angle

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