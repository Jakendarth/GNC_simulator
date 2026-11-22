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
    def __init__(self, name, ontology, x0, rng=None):
        self.rng = rng if rng is not None else np.random.default_rng(42)
        self.state = list(x0)
        self.extended_state = [x for i, x in enumerate(x0) if i != 5]
        self.extended_state = self.extended_state + [0] * 15
        self.y = list(x0)
        self.e = []
        self.ehat = []
        self.f = []
        self.alpha = 0
        self.beta = []
        self.tf = []
        self.psihat = []
        self.betahat = []
        self.uhat = []
        self.fhatu = []
        self.fhatx = []
        self.fhatpsi = []
        self.d0 = [0, 0, 0, 0, 0, 0, 0]
        self.d = [0, 0, 0, 0, 0, 0, 0]
        self.Td = [inf] * 6
        self.Omega = [0, 0, 0, 0, 0, 0]
        self.Fhat_history = pd.DataFrame({'f1': [], 'f2': [], 'f3': [], 'f4': [], 'f5': [], 'f6': []})
        self.z = 0
        self.u = [0]
        self.head_on_upperbound = []
        self.head_on_lowerbound = []
        self.ratio_head_on = []
        self.cond1 = []
        self.chi = [0]
        self.tau = 0.02
        self.usafe = []
        self.time = 0
        self.timestep = 1
        self.times = [self.time]
        self.route = None
        self.role = "Stand-On"
        self.error = 0
        self.arrived = 0
        self.ratio_time = []
        self.MPC = None
        self.timehat = []
        self.simulator = None
        self.estimator = None
        self.scenario = "Safe"
        self.positioning_noise = [1, 1, np.radians(2)]
        self.droute = (np.asarray([0] * 100), np.asarray([0] * 100))
        self.wp_idx = 0  # Initialize waypoint index
        self.psi_d_prev = x0[-1]  # Initialize previous desired heading
        self.yi_prev = 0
        self.sum_error = 0  # Initialize sum of heading errors
        self.error_old = 0  # Initialize previous heading error
        self.pass_angle_threshold = 90  # Angle threshold for passing a waypoint segment
        self.R_a = 5
        self.control = 0  # Initialize delta
        self.aggregated_error = 0
        self.measurement_noise = 0
        self.type = name
        if (self.type == "Own"):
            self.ontology = ontology.Own
        elif (self.type == "Other1"):
            self.ontology = ontology.Other1
        elif (self.type == "Other2"):
            self.ontology = ontology.Other2
        elif (self.type == "Other3"):
            self.ontology = ontology.Other3
        else:
            self.ontology = ontology.Other4

        self.distance_from_bank1 = 10**(-3)
        self.distance_from_bank2 = 10**(-3)

        self.log_t = []
        self.log_x_true = []
        self.log_y_true = []
        self.log_x_noisy = []
        self.log_y_noisy = []
        self.log_psi = []
        self.log_psi_noisy = []
        self.log_psi_d = []
        self.log_SOG = []
        self.log_delta = []
        self.log_dls = []
        self.log_dlp = []
        self.log_y_d = []

        # Initial noise conditions
        self.current_noise_x = 0.0
        self.current_noise_y = 0.0
        self.current_noise_psi = 0.0

        # Initial wind parameters
        self.wind_speed_nom = 0.0
        self.wind_dir_nom = 0.0
        self.wind_speed_noise = 0.0
        self.wind_dir_noise = 0.0
        self.current_V_w = 0.0
        self.current_psi_w = 0.0
        
        # Initial current parameters
        self.current_speed_nom = 0.0
        self.current_dir_nom = 0.0
        self.current_noise_sigma = 0.0
        self.current_V_c = 0.0
        self.current_alpha_c = 0.0

        # Initial river/current components in earth-fixed frame
        self.current_v_cx = 0.0
        self.current_v_cy = 0.0

        # Initial wave parameters
        self.wave_height_nom = 0.0
        self.wave_dir_nom = 0.0
        self.wave_height_noise = 0.0
        self.wave_dir_noise = 0.0
        self.current_Hs = 0.0
        self.current_chi_w = 0.0
        
        # Wave drift coefficients (assumed for 11BP pusher-barge)
        self.wave_cx = 0.2  # c_x
        self.wave_cy = 0.2  # c_y
        self.wave_cn = 0.0  # c_n
        
        # Aerodynamic coefficients (ITTC first-order Fourier for 11BP pusher-barge)
        self.rho_a = 1.225
        self.A_T = 64.0    # Transverse projected windage area [m^2]
        self.A_L = 252.0   # Lateral projected windage area [m^2]
        self.a_x0 = 0.0
        self.a_x1 = -1.05
        self.a_y1 = 1.20
        self.a_n1 = 0.25

    def include_sensor_noise(self, noise):
        self.measurement_noise = noise

    def include_sensor_faults(self, f, tf):
        self.f = f
        self.tf = tf
        self.Td = tf

    def dynamic_ontology_update(self, mystate, reference, mytime, start_time):
        t1_len = len(mytime.tolist())
        usq = [float(i)**2 for i in mystate[0]]
        vsq = [float(i)**2 for i in mystate[1]]
        Usq = [sum(x) for x in zip(usq, vsq)]

        # Wrap noisy heading to [-pi, pi) per Eq. (3.8)
        psi_noisy_vals = [
            float((p1 + self.current_noise_psi + np.pi) % (2 * np.pi) - np.pi)
            for p1 in mystate[-1]
        ]
        sog_noisy_vals = [
            float(np.sqrt(i) * (1.0 + self.measurement_noise * self.rng.uniform(-1.0, 1.0)))
            for i in Usq
        ]

        # Update ontology
        self.ontology.x.extend([float(x1) for x1 in mystate[3]])
        self.ontology.y.extend([float(y1) for y1 in mystate[4]])
        self.ontology.yx.extend([float(x2 + self.current_noise_x) for x2 in mystate[3]])
        self.ontology.yy.extend([float(y2 + self.current_noise_y) for y2 in mystate[4]])
        self.ontology.ypsi.extend(psi_noisy_vals)
        self.ontology.psi.extend([float(p2) for p2 in mystate[-1]])
        self.ontology.psi_d.extend([float(reference)] * t1_len)
        self.ontology.SOG.extend(sog_noisy_vals)
        self.ontology.rudder_angle.extend([float(self.u[-1])] * t1_len)
        self.ontology.starboard_distance.extend([float(self.distance_from_bank1)] * t1_len)
        self.ontology.port_distance.extend([float(self.distance_from_bank2)] * t1_len)
        self.ontology.time.extend([float(tt + start_time) for tt in mytime])
        self.time = start_time

        # Update CSV logs
        self.log_t.extend([float(tt + start_time) for tt in mytime])
        self.log_x_true.extend([float(x2) for x2 in mystate[3]])
        self.log_y_true.extend([float(y2) for y2 in mystate[4]])

        # Compute actual planned reference path y_d(x) from route + droute
        if self.route is not None and len(self.route[0]) > 1:
            x_ref = np.asarray(self.route[0], dtype=float)
            y_ref = np.asarray(self.route[1], dtype=float)
            if self.droute is not None and len(self.droute[0]) == len(x_ref):
                x_ref = x_ref + np.asarray(self.droute[0], dtype=float)
                y_ref = y_ref + np.asarray(self.droute[1], dtype=float)
            order = np.argsort(x_ref)
            x_ref_sorted = x_ref[order]
            y_ref_sorted = y_ref[order]
            self.log_y_d.extend([float(np.interp(x2, x_ref_sorted, y_ref_sorted)) for x2 in mystate[3]])
        else:
            self.log_y_d.extend([float(self.state[4])] * t1_len)

        # Measured GPS position (with noise)
        self.log_x_noisy.extend([float(x2 + self.current_noise_x) for x2 in mystate[3]])
        self.log_y_noisy.extend([float(y2 + self.current_noise_y) for y2 in mystate[4]])

        # True physical heading & noisy heading measurement
        self.log_psi.extend([float(p1) for p1 in mystate[-1]])
        self.log_psi_noisy.extend(psi_noisy_vals)
        self.log_psi_d.extend([float(reference)] * t1_len)
        self.log_SOG.extend(sog_noisy_vals)
        self.log_delta.extend([float(self.u[-1])] * t1_len)
        self.log_dls.extend([float(self.distance_from_bank1)] * t1_len)
        self.log_dlp.extend([float(self.distance_from_bank2)] * t1_len)

    def check_for_wp_segment_switch(self, wp_segment, d_0wp):
        return check_for_wp_segment_switch(self, wp_segment, d_0wp)

    def normalize_vec(self, vec):
        return normalize_vec(self, vec)

    def wrap_angle_to_pmpi(self, angle):
        return wrap_angle_to_pmpi(self, angle)
    
    def PID_controller(self, psi_d, dt):
        return PID_controller(self, psi_d, dt)
    
    def MPC_controller(self, psi_d):
        self.MPC = MPC_controller(self, psi_d)
        return MPC_controller(self, psi_d)

    def make_MPC_step(self):
        return self.MPC.make_step(np.array(self.y[3:5] + self.y[6:] + self.y[0:3]).reshape(-1, 1))

    def reset(self):
        self.state = [2.5, 0, 0, 0, 0, 0, 0]
        self.u = [[[300 / 60], [0], [0.328]]]
        self.y = [2.5, 0, 0, 0, 0, 0, 0]
        self.z = 0

    def calculate_derivatives(self):
        self.ontology.Yvdot = (1025 * self.ontology.L[0]**(3) / 2) * (-np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 + 0.16 * self.ontology.Cb[0] * self.ontology.B[0] / self.ontology.Td[0] - 5.1 * (self.ontology.B[0] / self.ontology.L[0])**2))
        self.ontology.Nvdot = (1025 * self.ontology.L[0]**(4) / 2) * (-np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (-0.0041 * self.ontology.B[0] / self.ontology.Td[0] + 1.1 * (self.ontology.B[0] / self.ontology.L[0])))
        self.ontology.Yv = (-np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 + 0.4 * self.ontology.Cb[0] * self.ontology.B[0] / self.ontology.Td[0]))
        self.ontology.Yrdot = (1025 * self.ontology.L[0]**(4) / 2) * (-np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 + 0.16 * self.ontology.Cb[0] * self.ontology.B[0] / self.ontology.Td[0] - 5.1 * (self.ontology.B[0] / self.ontology.L[0])**2))

    def sensor_output(self):
        # 1. Generate sensor noise for this time step using vessel's seeded RNG
        self.current_noise_x = self.rng.normal(loc=0.0, scale=self.positioning_noise[0])
        self.current_noise_y = self.rng.normal(loc=0.0, scale=self.positioning_noise[1])
        self.current_noise_psi = self.rng.normal(loc=0.0, scale=self.positioning_noise[2])

        # Get current true position
        x_pos = self.state[3]
        y_pos = self.state[4]

        # --- Determine the environment (sea vs inland) for Wind, Waves and Current ---
        if x_pos >= 3000.0:
            # Inland waterway (x >= 3000 m)
            shelter_factor = 0.4  # Wind sheltered by banks/buildings
            wave_factor = 0.0     # Negligible waves in inland waterways

            # Parabolic river velocity profile centered at canal centerline (y = -2000 m, width = 750 m)
            # Negative x-direction represents river flowing out towards the open sea (x < 3000 m)
            w = 750.0
            y_local = y_pos - (-2000.0)
            V_max = self.current_speed_nom
            v_cx_nom = -max(0.0, V_max * (1.0 - (2.0 * y_local / w)**2))
            v_cy_nom = 0.0
        else:
            # Open sea (x < 3000 m)
            shelter_factor = 1.0  # Full wind exposure
            wave_factor = 1.0     # Waves active in open sea
            
            # Uniform ocean current
            v_cx_nom = self.current_speed_nom * np.cos(self.current_dir_nom)
            v_cy_nom = self.current_speed_nom * np.sin(self.current_dir_nom)

        # --- Apply Wind ---
        V_w_raw = (self.wind_speed_nom * shelter_factor) + self.rng.normal(loc=0.0, scale=self.wind_speed_noise)
        self.current_V_w = max(0.0, V_w_raw)
        
        psi_w_raw = self.wind_dir_nom + self.rng.normal(loc=0.0, scale=self.wind_dir_noise)
        self.current_psi_w = (psi_w_raw + np.pi) % (2 * np.pi) - np.pi

        # --- Apply Waves ---
        HS_raw = (self.wave_height_nom * wave_factor) + self.rng.normal(loc=0.0, scale=self.wave_height_noise)
        self.current_Hs = max(0.0, HS_raw)

        chi_w_raw = self.wave_dir_nom + self.rng.normal(loc=0.0, scale=self.wave_dir_noise)
        self.current_chi_w = (chi_w_raw + np.pi) % (2 * np.pi) - np.pi

        # --- Apply Current (with isotropic Gaussian noise) ---
        w_cx = self.rng.normal(loc=0.0, scale=self.current_noise_sigma)
        w_cy = self.rng.normal(loc=0.0, scale=self.current_noise_sigma)
        self.current_v_cx = v_cx_nom + w_cx
        self.current_v_cy = v_cy_nom + w_cy

        # 2. Update measured feedback vector (self.y)
        for ii in range(0, len(self.state)):
            if ii < 3:
                self.y[ii] = self.state[ii] * (1.0 + self.measurement_noise * self.rng.uniform(-1.0, 1.0))
            elif ii == 3:  # X-position
                self.y[ii] = self.state[ii] + self.current_noise_x
            elif ii == 4:  # Y-position
                self.y[ii] = self.state[ii] + self.current_noise_y
            elif ii == 5:  # Heading (psi) wrapped to [-pi, pi)
                self.y[ii] = (self.state[ii] + self.current_noise_psi + np.pi) % (2 * np.pi) - np.pi
            else:
                self.y[ii] = self.state[ii]

    def update_derivatives_inland(self, envr):
        k = 2 * self.ontology.Td[0] / self.ontology.L[0]
        ke = k / ((self.T / (2 * envr.H)) * k + ((np.pi * self.ontology.Td[0]) / (2 * envr.H)) / np.tan((np.pi * self.ontology.Td[0]) / (2 * envr.H))**0.7)
        self.ontology.Yr = np.pi / 4 * ke
        self.ontology.Nr = -0.54 * ke + ke**2
        ke = k / ((self.ontology.Td[0] / (2 * envr.H)) * k + ((np.pi * self.ontology.Td[0]) / (2 * envr.H)) / np.tan((np.pi * self.ontology.Td[0]) / (2 * envr.H))**2.3)
        self.ontology.Yv = -np.pi / 2 * ke - 1.4 * self.ontology.Cb[0] * self.ontology.B[0] / self.ontology.L[0]
        ke = k / ((self.ontology.Td[0] / (2 * envr.H)) * k + ((np.pi * self.ontology.Td[0]) / (2 * envr.H)) / np.tan((np.pi * self.ontology.Td[0]) / (2 * envr.H))**1.7)
        self.ontology.Nv = -ke

    def calculate_forces(self, v):
        # Define currents
        psi_current = v[5]
        u_current = v[0]  # SOG (Speed Over Ground)
        v_current = v[1]  # SOG
        r_current = v[2]

        # Transformation of earth-fixed current to Body-Fixed frame
        u_c = self.current_v_cx * np.cos(psi_current) + self.current_v_cy * np.sin(psi_current)
        v_c = -self.current_v_cx * np.sin(psi_current) + self.current_v_cy * np.cos(psi_current)

        # Calculate water-relative velocities
        u_rel = u_current - u_c
        v_rel = v_current - v_c
        U_rel = np.sqrt(u_rel**2 + v_rel**2)
        v_m_rel = v_rel - self.ontology.xg[0] * r_current

        #================================================
        # 1. Constants
        #================================================
        R0 = 0.0428
        t = 0.326
        wp0 = 0.576
        C1 = 0.98
        Dp = 1.8
        lp = (self.ontology.L[0] / 2 + self.ontology.xg[-1]) / (self.ontology.L[0])
        aH = 0.418
        lR = -1.113
        tR = 0.055
        xH = -0.189 * (self.ontology.L[0])
        gR = 0.293
        AR = 4
        e = 1.823
        k = 0.6 / e
        w = 0.3
        xR = -self.ontology.L[0] / 2
        wb = 4
        ys = self.distance_from_bank1
        yp = self.distance_from_bank2
        yB3 = 0.5 * self.ontology.B[0] * (1 / yp + 1 / ys)
        a_ikH = [-0.461 + 0.368 / self.ontology.Cb[0], -0.173 + 0.0114 * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0], 0.00634 - 0.00046 * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0], 1.36 - 0.0658 * self.ontology.L[0] / self.ontology.Td[0], 0.257 - 0.0896 * self.ontology.B[0] / self.ontology.Td[0], -0.0191 + 0.00634 * self.ontology.B[0] / self.ontology.Td[0]]
        b_ikH = [-0.119 + 2.11 * self.ontology.Td[0] / self.ontology.L[0], -0.0354 + 0.195 * self.ontology.B[0] / self.ontology.L[0], 2.29 * 10**(-4) - 6.92 * 10**(-5) * self.ontology.B[0] / self.ontology.Td[0], 0.134 - 2.35 * self.ontology.Td[0] / self.ontology.L[0], -4.23 * 10**(-2) + 2.93 * 10**(-3) * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0], 1.23 * 10**(-3) - 8.53 * 10**(-5) * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0]]
        a_iKP = [0.1 - 0.0524 * self.ontology.Td[0] / self.ontology.B[0], -0.0087 - 0.0386 * self.ontology.B[0] / self.ontology.L[0], -0.000105 - 0.000000923 * self.ontology.L[0] / self.ontology.B[0], 0.0592 - 0.0211 * self.ontology.B[0] / self.ontology.Td[0], -0.000282 - 0.000242 * self.ontology.L[0] / self.ontology.B[0], 0.000232 - 0.0000198 * self.ontology.L[0] / self.ontology.B[0]]
        b_iKP = [-0.00495 + 0.125 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.L[0], 0.00129 + 0.00537 * self.ontology.Td[0] / self.ontology.B[0], -0.000268 + 0.000878 * self.ontology.Td[0] / self.ontology.B[0], 0.00381 - 0.0129 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.B[0], 0.00344 - 0.0011 * self.ontology.B[0] / self.ontology.Td[0], 0.000488 - 0.00152 * self.ontology.Td[0] / self.ontology.B[0]]
        a_ikHP = [-0.237 + 0.422 * self.ontology.Td[0] / self.ontology.B[0], 0.0289 - 0.0022 * self.ontology.L[0] / self.ontology.B[0], -0.00657 + 0.00118 * self.ontology.L[0] / self.ontology.B[0], 0.419 - 0.0479 * self.ontology.L[0] / self.ontology.B[0], -0.0449 + 0.00998 * self.ontology.L[0] / self.ontology.B[0], 0.0106 + 0.0424 * self.ontology.Td[0] / self.ontology.B[0]]
        b_ikHP = [-0.289 + 5.71 * self.ontology.Td[0] / self.ontology.L[0], -0.122 + 0.353 * self.ontology.Td[0] / self.ontology.B[0], 0.00787 - 0.0233 * self.ontology.Td[0] / self.ontology.B[0], -0.0933 + 0.0157 * self.ontology.L[0] / self.ontology.B[0], 0.26 - 0.795 * self.ontology.Td[0] / self.ontology.B[0], -0.0199 + 0.0598 * self.ontology.Td[0] / self.ontology.B[0]]

        #================================================
        # 2. Tunable parameters by controller
        #================================================
        delta = self.u[-1]
        p = 0.328
        Np = 300 / 60

        #================================================
        # 3. Problem defined variables
        #================================================
        U = U_rel
        s = 1 - U / (Np * Dp)

        #================================================
        # 4. Hull forces calculation
        #================================================
        Fn = u_rel / np.sqrt(9.81 * self.ontology.L[0])
        bm = -np.arctan(v_m_rel / u_rel) if u_rel != 0 else 0.0
        r_nd = v[2] * self.ontology.L[0] / U
        try:
            XH = -R0 * np.cos(bm)**2 + self.ontology.Xbb[0] * bm**2 + self.ontology.Xbr[0] * bm * r_nd + self.ontology.Xrr[0] * r_nd**2
            YH = self.ontology.Yb[0] * bm + self.ontology.Yr * r_nd + self.ontology.Ybbb[0] * bm**3 + self.ontology.Ybbr[0] * bm**2 * r_nd + self.ontology.Ybrr[0] * bm * r_nd**2 + self.ontology.Yrrr[0] * r_nd**3
            NH = self.ontology.Nb[0] * bm + self.ontology.Nr * r_nd + self.ontology.Nbbb[0] * bm**3 + self.ontology.Nbbr[0] * bm**2 * r_nd + self.ontology.Nbrr[0] * bm * r_nd**2 + self.ontology.Nrrr[0] * r_nd**3
        except:
            XH = self.ontology.Xu * (v[0] - U) / U
            YH = self.ontology.Yv * v[1] / U + self.ontology.Yr * v[2] * self.ontology.L[0] / U
            NH = self.ontology.Nv * v[1] / U + self.ontology.Nr * v[2] * self.ontology.L[0] / U

        #================================================
        # 5. Propeller force calculation
        #================================================
        b = -np.arctan(v_rel / u_rel) if u_rel != 0 else 0.0
        bp = b - lp * r_nd
        wp = wp0 * np.exp(-C1 * bp**2)
        Jp = u_rel * (1 - wp) / (Np * Dp)
        KT = -0.326 * p * Jp - 0.2005 * Jp + 0.5234 * p - 0.0398
        XP = (2 * (1 - t) * 1025 * Np**2 * Dp**4 * KT) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2)

        #================================================
        # 6. Rudder forces calculation
        #================================================
        uP = u_rel * (1 - s)
        fa = 6.13 / (1 + 2.25)
        bR = b - lR * r_nd
        vR = v_rel * gR * bR
        eta = Dp / 2
        uR = e * uP / (1 - s) * np.sqrt(1 - 2 * (1 - eta * k) * s + (1 - eta * k * (2 - k)) * s**2)
        aR = delta - np.arctan(vR / uR)
        UR = np.sqrt(uR**2 + vR**2)
        FN = (0.5 * 1025 * AR * UR**2 * fa * np.sin(aR))
        XR = -(1 - tR) * 2 * FN * np.sin(delta) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2)
        YR = -(1 + aH) * 2 * FN * np.cos(delta) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2)
        NR = -(xR + aH * xH) * 2 * FN * np.cos(delta) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2)

        #================================================
        # 7. Bank effects calculation
        #================================================
        Uref = np.sqrt(np.abs((1025 * 0.25 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * XP / (1025 * np.pi * Dp**2 / 8)))
        h_eff = 1.2 * self.ontology.Td[0]
        T_hT = self.ontology.Td[0] / (h_eff - self.ontology.Td[0])
        Y_BH = sum(a * np.sign(yB3) * np.abs(yB3)**(i // 3 + 1) * T_hT**(i % 3) for i, a in enumerate(a_ikH)) * 0.5 * 1025 * self.ontology.L[0] * self.ontology.Td[0] * (v[0]**2) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2)
        N_BH = sum(b * np.sign(yB3) * np.abs(yB3)**(i // 3 + 1) * T_hT**(i % 3) for i, b in enumerate(b_ikH)) * 0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0] * (v[0]**2) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2)
        Y_BP = sum(p * np.sign(yB3) * np.abs(yB3)**(i // 3 + 1) * T_hT**(i % 3) for i, p in enumerate(a_iKP)) * 0.5 * 1025 * self.ontology.L[0] * self.ontology.Td[0] * (Uref**2) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2)
        N_BP = sum(q * np.sign(yB3) * np.abs(yB3)**(i // 3 + 1) * T_hT**(i % 3) for i, q in enumerate(b_iKP)) * 0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0] * (Uref**2) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2)
        YB = (sum(hp * np.sign(yB3) * np.abs(yB3)**(i // 3 + 1) * T_hT**(i % 3) for i, hp in enumerate(a_ikHP)) * 0.5 * 1025 * self.ontology.L[0] * self.ontology.Td[0] * Fn * Uref**2) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2)
        NB = (sum(k * np.sign(yB3) * np.abs(yB3)**(i // 3 + 1) * T_hT**(i % 3) for i, k in enumerate(b_ikHP)) * 0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0] * Fn * Uref**2) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2)

        #================================================
        # 8. Wind effects calculation
        #================================================
        v_m = v_current - self.ontology.xg[0] * r_current
        
        u_R = -u_current + self.current_V_w * np.cos(self.current_psi_w - psi_current)
        v_R = -v_m + self.current_V_w * np.sin(self.current_psi_w - psi_current)
        V_R = np.sqrt(u_R**2 + v_R**2)
        
        gamma_R = -np.arctan2(v_R, -u_R)
        
        C_X = self.a_x0 + self.a_x1 * np.cos(gamma_R)
        C_Y = self.a_y1 * np.sin(gamma_R)
        C_N = self.a_n1 * np.sin(gamma_R)
        
        X_F_wind = 0.5 * self.rho_a * V_R**2 * C_X * self.A_T
        Y_F_wind = 0.5 * self.rho_a * V_R**2 * C_Y * self.A_L
        N_F_wind = 0.5 * self.rho_a * V_R**2 * C_N * self.A_L * self.ontology.L[0]
        
        # Convert from standard frame (y starboard) to base model frame (y port) about midship.
        # Note: rhs() already performs the CoG moment shift (N_m = N_d - Y_d * xg) on total forces,
        # so xg * Y_wind is NOT added here to avoid double-counting.
        X_wind = X_F_wind
        Y_wind = -Y_F_wind
        N_wind = -N_F_wind
        
        if U > 0.01:
            denom_surge = 1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2
            denom_yaw = 1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2
            
            XH += X_wind / denom_surge
            YH += Y_wind / denom_surge
            NH += N_wind / denom_yaw

        #================================================
        # 9. Wave drift forces calculation
        #================================================
        rho_w = 1025.0
        g = 9.81
        L = self.ontology.L[0]
        B = self.ontology.B[0]
        
        # Relative wave angle wrapped to [-pi, pi)
        mu = (self.current_chi_w - psi_current + np.pi) % (2 * np.pi) - np.pi
        zeta_a_sq = (self.current_Hs**2) / 8.0
        
        # Wave drift forces defined about CoG per Eq. (3.29)
        X_wave = 0.5 * rho_w * g * zeta_a_sq * B * self.wave_cx * np.cos(mu)
        Y_wave = 0.5 * rho_w * g * zeta_a_sq * L * self.wave_cy * np.sin(mu)
        N_wave_cog = 0.5 * rho_w * g * zeta_a_sq * (L**2) * self.wave_cn * np.sin(mu)
        
        # Pre-compensate for rhs()'s (-Y_d * xg) shift so net yaw moment about CoG remains N_wave_cog
        N_wave_mid = N_wave_cog + self.ontology.xg[0] * Y_wave

        if U > 0.01:
            denom_surge = 1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2
            denom_yaw = 1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2
            
            XH += X_wave / denom_surge
            YH += Y_wave / denom_surge
            NH += N_wave_mid / denom_yaw

        return XH, YH, NH, XP, XR, YR, NR, YB + Y_BH + Y_BP, NB + N_BH + N_BP, U

    def rhs(self, s, v, tend=1000): 
        XH, YH, NH, XP, XR, YR, NR, YB, NB, U = self.calculate_forces(v)
        Am = self.ontology.B[0] * self.ontology.Td[0]
        h = 1.2 * self.ontology.Td[0]
        beff = 5 * self.ontology.B[0]
        Ac = beff * h
        self.z = 0.133 * self.ontology.Cb[0] * (Am / (Ac - Am))**(2 / 3) * U**2.08
        XH = XH * self.ontology.force_switch[0]
        YH = YH * self.ontology.force_switch[0]
        NH = NH * self.ontology.force_switch[0]
        XP = XP * self.ontology.force_switch[1]
        XR = XR * self.ontology.force_switch[2]
        YR = YR * self.ontology.force_switch[2]
        NR = NR * self.ontology.force_switch[2]
        YB = YB * self.ontology.force_switch[3]
        NB = NB * self.ontology.force_switch[3]
        m26_nd = 0.0108
        m11_nd = 0.0195
        m22_nd = 0.3722
        m66_nd = 0.0124
        m11 = m11_nd * (0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0])
        m22 = m22_nd * (0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0])
        m26 = m26_nd * (0.5 * 1025 * self.ontology.L[0]**3 * self.ontology.Td[0])
        m66 = m66_nd * (0.5 * 1025 * self.ontology.L[0]**4 * self.ontology.Td[0])
        X_d = (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * (XP + XR) + (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * XH
        Y_d = (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * (YR + YB) + (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * YH
        N_d = (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2) * (NR + NB) + (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2) * NH
        Y_m = Y_d - (self.ontology.m[0] + m11) * v[0] * v[2]
        N_m = N_d - Y_d * self.ontology.xg[0]
        coeff_matrix = np.array([[self.ontology.m[0] + m22, m26], [m26, self.ontology.Iz[0] + m66]])
        vdot_num_matrix = np.array([[Y_m, m26], [N_m, self.ontology.Iz[0] + m66]])
        rdot_num_matrix = np.array([[self.ontology.m[0] + m22, Y_m], [m26, N_m]])
        rates_change_modified = [
            (X_d + (self.ontology.m[0] + m22) * v[1] * v[2]) / (self.ontology.m[0] + m11),
            (np.linalg.det(vdot_num_matrix)) / (np.linalg.det(coeff_matrix)),
            (np.linalg.det(rdot_num_matrix)) / (np.linalg.det(coeff_matrix)),
            v[0] * np.cos(v[5]) - v[1] * np.sin(v[5]),
            v[0] * np.sin(v[5]) + v[1] * np.cos(v[5]),
            v[2]
        ]
        return rates_change_modified

    def eso_rhs(self, s, v, tend=1000):
        L = 3 * np.ones(6)
        Gamma = [400, 400, 400, 400, 400, 400, 5 * 10**(-6)]
        XH, YH, NH, XP, XR, YR, NR, YB, NB, U = self.calculate_forces(v)
        Am = self.ontology.B[0] * self.ontology.Td[0]
        h = 1.2 * self.ontology.Td[0]
        beff = 5 * self.ontology.B[0]
        Ac = beff * h
        self.z = 0.133 * self.ontology.Cb[0] * (Am / (Ac - Am))**(2 / 3) * U**2.08
        a_1, z_1, a_d1, z_d1, x1_bar0 = 2, 2, 3, 2, 4
        d1_bar = 0.03 * x1_bar0
        a_4, z_4, a_d4, z_d4, x4_bar0 = 2, 2, 3, 2, 8000
        d4_bar = 0.03 * x4_bar0
        a_6, z_6, a_d6, z_d6, x6_bar0 = 2, 2, 3, 2, 1.2
        d6_bar = 0.03 * x6_bar0
        Y_1 = a_1 * x1_bar0 * np.exp(-z_1 * self.time) + a_d1 * d1_bar * (1 - np.exp(-z_d1 * self.time)) / z_d1
        Y_4 = a_4 * x4_bar0 * np.exp(-z_4 * self.time) + a_d4 * d4_bar * (1 - np.exp(-z_d4 * self.time)) / z_d4
        Y_6 = a_6 * x6_bar0 * np.exp(-z_6 * self.time) + a_d6 * d6_bar * (1 - np.exp(-z_d6 * self.time)) / z_d6
        XH = XH * self.ontology.force_switch[0]
        YH = YH * self.ontology.force_switch[0]
        NH = NH * self.ontology.force_switch[0]
        XP = XP * self.ontology.force_switch[1]
        XR = XR * self.ontology.force_switch[2]
        YR = YR * self.ontology.force_switch[2]
        NR = NR * self.ontology.force_switch[2]
        YB = YB * self.ontology.force_switch[3]
        NB = NB * self.ontology.force_switch[3]
        m26_nd = 0.0108
        m11_nd = 0.0195
        m22_nd = 0.3722
        m66_nd = 0.0124
        m11 = m11_nd * (0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0])
        m22 = m22_nd * (0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0])
        m26 = m26_nd * (0.5 * 1025 * self.ontology.L[0]**3 * self.ontology.Td[0])
        m66 = m66_nd * (0.5 * 1025 * self.ontology.L[0]**4 * self.ontology.Td[0])
        X_d = (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * (XP + XR) + (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * XH
        Y_d = (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * (YR + YB) + (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * YH
        N_d = (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2) * (NR + NB) + (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2) * NH
        Y_m = Y_d - (self.ontology.m[0] + m11) * v[0] * v[2]
        N_m = N_d - Y_d * self.ontology.xg[0]
        coeff_matrix = np.array([[self.ontology.m[0] + m22, m26], [m26, self.ontology.Iz[0] + m66]])
        vdot_num_matrix = np.array([[Y_m, m26], [N_m, self.ontology.Iz[0] + m66]])
        rdot_num_matrix = np.array([[self.ontology.m[0] + m22, Y_m], [m26, N_m]])
        rates_change = [
            (X_d + (self.ontology.m[0] + m22) * v[1] * v[2]) / (self.ontology.m[0] + m11) + L[0] * (self.y[0] - v[0] - v[12]) + v[6] * Gamma[0] * (self.y[0] - v[0] - v[12]) * (v[6] + 1) * self.d[0] * int(self.Td[0] <= self.time) * int(np.abs(self.y[0] - v[0] - v[12]) > Y_1),
            (np.linalg.det(vdot_num_matrix)) / (np.linalg.det(coeff_matrix)) + L[1] * (self.y[1] - v[1] - v[13]) + v[7] * Gamma[1] * (self.y[1] - v[1] - v[13]) * (v[7] + 1) * self.d[1] * int(self.Td[1] <= self.time),
            (np.linalg.det(rdot_num_matrix)) / (np.linalg.det(coeff_matrix)) + L[2] * (self.y[2] - v[2] - v[14]) + v[8] * Gamma[2] * (self.y[2] - v[2] - v[14]) * (v[8] + 1) * self.d[2] * int(self.Td[2] <= self.time),
            v[0] * np.cos(v[5]) - v[1] * np.sin(v[5]) + L[3] * (self.ontology.yx[-1] - v[3] - v[15]) + v[9] * Gamma[3] * (self.ontology.yx[-1] - v[3] - v[15]) * (v[9] + 1) * self.d[3] * int(np.abs(self.ontology.yx[-1] - v[3] - v[15]) > Y_4),
            v[0] * np.sin(v[5]) + v[1] * np.cos(v[5]) + L[4] * (self.ontology.yy[-1] - v[4] - v[16]) + v[10] * Gamma[4] * (self.ontology.yy[-1] - v[4] - v[16]) * (v[10] + 1) * self.d[4] * int(self.Td[4] <= self.time),
            v[2] + L[5] * (self.ontology.ypsi[-1] - v[5] - v[17]) + v[11] * Gamma[5] * (self.ontology.ypsi[-1] - v[5] - v[17]) * (v[11] + 1) * self.d[-1] * int(np.abs(self.ontology.ypsi[-1] - v[5] - v[17]) > Y_6),
            -L[0] * v[6] - L[0] * int(self.Td[0] <= self.time) * int(np.abs(self.y[0] - v[0] - v[12]) > Y_1),
            -L[1] * v[7] - L[1] * int(self.Td[1] <= self.time),
            -L[2] * v[8] - L[2] * int(self.Td[2] <= self.time),
            -L[3] * v[9] - L[3] * int(self.Td[3] <= self.time) * int(np.abs(self.ontology.yx[-1] - v[3] - v[15]) > Y_4),
            -L[4] * v[10] - L[4] * int(self.Td[4] <= self.time),
            -L[5] * v[11] - L[5] * int(self.Td[-1] <= self.time) * int(np.abs(self.ontology.ypsi[-1] - v[5] - v[17]) > Y_6),
            Gamma[0] * (self.ontology.SOG[-1] * np.cos(v[5]) - v[0] - v[12]) * (v[6] + 1) * self.d[0] * int(self.Td[0] <= self.time) * int(np.abs(self.y[0] - v[0] - v[12]) > Y_1),
            Gamma[1] * (self.ontology.SOG[-1] * np.sin(v[5]) - v[1] - v[13]) * (v[7] + 1) * self.d[1] * int(self.Td[1] <= self.time) * int(self.y[1] - v[1] - v[13] > self.measurement_noise * v[1]),
            Gamma[2] * (self.y[2] - v[2] - v[14]) * (v[8] + 1) * self.d[2] * int(self.Td[2] <= self.time) * int(self.y[2] - v[2] - v[14] > self.measurement_noise * v[2]),
            Gamma[3] * (self.ontology.yx[-1] - v[3] - v[15]) * (v[9] + 1) * self.d[3] * int(self.Td[3] <= self.time) * int(np.abs(self.y[3] - v[3] - v[15]) > Y_4),
            Gamma[4] * (self.ontology.yy[-1] - v[4] - v[16]) * (v[10] + 1) * self.d[4] * int(self.Td[4] <= self.time) * int(self.y[4] - v[4] - v[16] > self.measurement_noise * v[4]),
            Gamma[5] * (self.ontology.ypsi[-1] - v[5] - v[17]) * (v[11] + 1) * self.d[-1] * int(self.Td[-1] <= self.time) * int(np.abs(self.y[-1] - v[5] - v[17]) > Y_6),
            Gamma[6] * (200 * self.ontology.SOG[-1]) / np.sqrt((200)**2 + (v[20] + 200 * v[18])**2) * v[20],
            self.ontology.SOG[-1] * np.sin(v[5] - self.alpha + v[18]),
            self.ontology.SOG[-1] * np.sin(v[5] - self.alpha + self.beta[-1])
        ]
        return rates_change

    def differential_algebraic_model(self, tend):
        self.ontology.m[0] = 1025 * self.ontology.Cb[0] * self.ontology.L[0] * self.ontology.B[0] * self.ontology.Td[0]
        self.ontology.mx[0] = 0.006 * (0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0])
        self.ontology.my[0] = 0.0929 * (0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0])
        self.ontology.Iz[0] = (1 / 12) * self.ontology.m[0] * (self.ontology.L[0]**2 + self.ontology.B[0]**2)
        self.ontology.Jz[0] = 0.0049 * (0.5 * 1025 * self.ontology.L[0]**4 * self.ontology.Td[0])
        res = solve_ivp(
            self.rhs, t_span=(0, tend),
            y0=[x for i, x in enumerate(self.state) if i != 5],
            method='RK45', rtol=1e-8, atol=1e-8
        )
        self.state = [res.y[ii][-1] for ii in range(len(res.y))]
        self.sensor_output()
        self.state.insert(5, self.z)
        return res.t, res.y


class environment:
    def __init__(self, shapefile, width, depth):
        self.width = width    # Width of the waterway
        self.depth = depth    # Depth of the waterway
        self.shapefile = shapefile  # Shapefile of the waterway

    def plot_environment(self):
        fig, ax = plt.subplots(figsize=(10, 10))
        for i, segment in enumerate(self.shapefile.iloc[0:1].geometry):
            x, y = segment.xy
        ax.plot(x, y, linewidth=1, color='red', linestyle='--')
        ax.set_title('Zandkreek en Veerse Meer')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.show()