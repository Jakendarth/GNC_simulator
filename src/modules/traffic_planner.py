import do_mpc as mpc
import casadi
import sys
import numpy as np
sys.path.insert(1, 'src/')

from channel import *
from intel_guidance import *

def calculate_forces(self, v, delta, Np, depth, sinkage, yB3_k):
    R0 = 0.0428
    t = 0.326
    wp0 = 0.576
    p = 0.328
    C1 = 0.98
    Dp = 1.8
    lp = (self.ontology.L[0] / 2 + self.ontology.xg[0]) / (self.ontology.L[0])
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
    yB3 = yB3_k
    a_ikH = [-0.461 + 0.368 / self.ontology.Cb[0], -0.173 + 0.0114 * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0], 0.00634 - 0.00046 * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0], 1.36 - 0.0658 * self.ontology.L[0] / self.ontology.Td[0], 0.257 - 0.0896 * self.ontology.B[0] / self.ontology.Td[0], -0.0191 + 0.00634 * self.ontology.B[0] / self.ontology.Td[0]]
    b_ikH = [-0.119 + 2.11 * self.ontology.Td[0] / self.ontology.L[0], -0.0354 + 0.195 * self.ontology.B[0] / self.ontology.L[0], 2.29 * 10**(-4) - 6.92 * 10**(-5) * self.ontology.B[0] / self.ontology.Td[0], 0.134 - 2.35 * self.ontology.Td[0] / self.ontology.L[0], -4.23 * 10**(-2) + 2.93 * 10**(-3) * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0], 1.23 * 10**(-3) - 8.53 * 10**(-5) * self.ontology.Cb[0] * self.ontology.L[0] / self.ontology.Td[0]]
    a_iKP = [0.1 - 0.0524 * self.ontology.Td[0] / self.ontology.B[0], -0.0087 - 0.0386 * self.ontology.B[0] / self.ontology.L[0], -0.000105 - 0.000000923 * self.ontology.L[0] / self.ontology.B[0], 0.0592 - 0.0211 * self.ontology.B[0] / self.ontology.Td[0], -0.000282 - 0.000242 * self.ontology.L[0] / self.ontology.B[0], 0.000232 - 0.0000198 * self.ontology.L[0] / self.ontology.B[0]]
    b_iKP = [-0.00495 + 0.125 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.L[0], 0.00129 + 0.00537 * self.ontology.Td[0] / self.ontology.B[0], -0.000268 + 0.000878 * self.ontology.Td[0] / self.ontology.B[0], 0.00381 - 0.0129 * self.ontology.Cb[0] * self.ontology.Td[0] / self.ontology.B[0], 0.00344 - 0.0011 * self.ontology.B[0] / self.ontology.Td[0], 0.000488 - 0.00152 * self.ontology.Td[0] / self.ontology.B[0]]
    a_ikHP = [-0.237 + 0.422 * self.ontology.Td[0] / self.ontology.B[0], 0.0289 - 0.0022 * self.ontology.L[0] / self.ontology.B[0], -0.00657 + 0.00118 * self.ontology.L[0] / self.ontology.B[0], 0.419 - 0.0479 * self.ontology.L[0] / self.ontology.B[0], -0.0449 + 0.00998 * self.ontology.L[0] / self.ontology.B[0], 0.0106 + 0.0424 * self.ontology.Td[0] / self.ontology.B[0]]
    b_ikHP = [-0.289 + 5.71 * self.ontology.Td[0] / self.ontology.L[0], -0.122 + 0.353 * self.ontology.Td[0] / self.ontology.B[0], 0.00787 - 0.0233 * self.ontology.Td[0] / self.ontology.B[0], -0.0933 + 0.0157 * self.ontology.L[0] / self.ontology.B[0], 0.26 - 0.795 * self.ontology.Td[0] / self.ontology.B[0], -0.0199 + 0.0598 * self.ontology.Td[0] / self.ontology.B[0]]

    U = casadi.sqrt(casadi.power((v[0] - 0.02), 2) + casadi.power((v[1] - 0.01), 2))
    s = 1 - U / (Np * Dp)
    Fn = v[0] / casadi.sqrt(9.81 * self.ontology.L[0])
    bm = -casadi.arctan((v[1] - self.ontology.xg[0] * v[2]) / v[0])
    r_nd = v[2] * self.ontology.L[0] / U
    try:
        XH = -R0 * casadi.power(casadi.cos(bm), 2) + self.ontology.Xbb[0] * casadi.power(bm, 2) + self.ontology.Xbr[0] * bm * r_nd + self.ontology.Xrr[0] * casadi.power(r_nd, 2)
        YH = self.ontology.Yb[0] * bm + self.ontology.Yr * r_nd + self.ontology.Ybbb[0] * casadi.power(bm, 3) + self.ontology.Ybbr[0] * casadi.power(bm, 2) * r_nd + self.ontology.Ybrr[0] * bm * casadi.power(r_nd, 2) + self.ontology.Yrrr[0] * casadi.power(r_nd, 3)
        NH = self.ontology.Nb[0] * bm + self.ontology.Nr * r_nd + self.ontology.Nbbb[0] * casadi.power(bm, 3) + self.ontology.Nbbr[0] * casadi.power(bm, 2) * r_nd + self.ontology.Nbrr[0] * bm * casadi.power(r_nd, 2) + self.ontology.Nrrr[0] * casadi.power(r_nd, 3)
    except:
        XH = self.ontology.Xu * (v[0] - U) / U
        YH = self.ontology.Yv * v[1] / U + self.ontology.Yr * v[2] * self.ontology.L[0] / U
        NH = self.ontology.Nv * v[1] / U + self.ontology.Nr * v[2] * self.ontology.L[0] / U

    b = -casadi.arctan((v[1]) / v[0])
    bp = b - lp * r_nd
    wp = wp0 * casadi.exp(-C1 * casadi.power(bp, 2))
    Jp = v[0] * (1 - wp) / (Np * Dp)
    KT = -0.326 * p * Jp - 0.2005 * Jp + 0.5234 * p - 0.0398
    XP = (2 * (1 - t) * 1025 * casadi.power(Np, 2) * casadi.power(Dp, 4) * KT) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2))

    delta_rad = delta
    uP = v[0] * (1 - s)
    fa = 6.13 / (1 + 2.25)
    bR = b - lR * r_nd
    vR = v[1] * gR * bR
    eta = Dp / 2
    uR = e * uP / (1 - s) * casadi.sqrt(1 - 2 * (1 - eta * k) * s + (1 - eta * k * (2 - k)) * casadi.power(s, 2))
    aR = delta_rad - casadi.arctan(vR / uR)
    UR = casadi.sqrt(casadi.power(uR, 2) + casadi.power(vR, 2))
    FN = (0.5 * 1025 * AR * casadi.power(UR, 2) * fa * casadi.sin(aR))
    XR = -(1 - tR) * 2 * FN * casadi.sin(delta_rad) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2))
    YR = -(1 + aH) * 2 * FN * casadi.cos(delta_rad) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2))
    NR = -(xR + aH * xH) * 2 * FN * casadi.cos(delta_rad) / (1025 * 0.5 * casadi.power(self.ontology.L[0], 2) * self.ontology.Td[0] * casadi.power(U, 2))

    Uref = casadi.sqrt(casadi.fabs((1025 * 0.25 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2)) * XP / (1025 * casadi.pi * casadi.power(Dp, 2 / 8))))
    h_eff = depth - sinkage
    T_hT = self.ontology.Td[0] / (h_eff - self.ontology.Td[0])
    Y_BH = sum(a * casadi.sign(yB3) * casadi.power(casadi.fabs(yB3), (i // 3 + 1)) * casadi.power(T_hT, (i % 3)) for i, a in enumerate(a_ikH)) * 0.5 * 1025 * self.ontology.L[0] * self.ontology.Td[0] * (casadi.power(v[0], 2)) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2))
    N_BH = sum(b * casadi.sign(yB3) * casadi.power(casadi.fabs(yB3), (i // 3 + 1)) * casadi.power(T_hT, (i % 3)) for i, b in enumerate(b_ikH)) * 0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0] * (casadi.power(v[0], 2)) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * casadi.power(U, 2))
    Y_BP = sum(p * casadi.sign(yB3) * casadi.power(casadi.fabs(yB3), (i // 3 + 1)) * casadi.power(T_hT, (i % 3)) for i, p in enumerate(a_iKP)) * 0.5 * 1025 * self.ontology.L[0] * self.ontology.Td[0] * (casadi.power(Uref, 2)) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2))
    N_BP = sum(q * casadi.sign(yB3) * casadi.power(casadi.fabs(yB3), (i // 3 + 1)) * casadi.power(T_hT, (i % 3)) for i, q in enumerate(b_iKP)) * 0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0] * (casadi.power(Uref, 2)) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * casadi.power(U, 2))
    YB = (sum(hp * casadi.sign(yB3) * casadi.power(casadi.fabs(yB3), (i // 3 + 1)) * casadi.power(T_hT, (i % 3)) for i, hp in enumerate(a_ikHP)) * 0.5 * 1025 * self.ontology.L[0] * self.ontology.Td[0] * Fn * casadi.power(Uref, 2)) / (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * casadi.power(U, 2))
    NB = (sum(k * casadi.sign(yB3) * casadi.power(casadi.fabs(yB3), (i // 3 + 1)) * casadi.power(T_hT, (i % 3)) for i, k in enumerate(b_ikHP)) * 0.5 * 1025 * self.ontology.L[0]**2 * self.ontology.Td[0] * Fn * casadi.power(Uref, 2)) / (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * casadi.power(U, 2))
    return XH, YH, NH, XP, XR, YR, NR, YB + Y_BH + Y_BP, NB + N_BH + N_BP, U

def MPC_controller(myvessel, psi_d):
    MPC_prediction_horizon = 5
    MPC_time_step = 0.5
    model_type = 'continuous'
    model = mpc.model.Model(model_type)
    x_1 = model.set_variable(var_type='_x', var_name='x_1', shape=(1, 1))
    x_2 = model.set_variable(var_type='_x', var_name='x_2', shape=(1, 1))
    x_3 = model.set_variable(var_type='_z', var_name='x_3', shape=(1, 1))
    x_4 = model.set_variable(var_type='_x', var_name='x_4', shape=(1, 1))
    x_5 = model.set_variable(var_type='_x', var_name='x_5', shape=(1, 1))
    x_6 = model.set_variable(var_type='_x', var_name='x_6', shape=(1, 1))
    x_7 = model.set_variable(var_type='_x', var_name='x_7', shape=(1, 1))
    Np = model.set_variable(var_type='_u', var_name='Np', shape=(1, 1))
    delta = model.set_variable(var_type='_u', var_name='delta', shape=(1, 1))
    psi_ref = model.set_variable(var_type='_tvp', var_name='psi_ref', shape=(1, 1))

    model.set_rhs('x_1', x_5 * casadi.cos(x_4) - x_6 * casadi.sin(x_4))
    model.set_rhs('x_2', x_5 * casadi.sin(x_4) + x_6 * casadi.cos(x_4))
    v_sym = casadi.vertcat(x_5, x_6, x_7, x_1, x_2, x_4, x_3)
    self = myvessel
    u_now_x = self.y[0] * np.cos(self.y[-1]) - self.y[1] * np.sin(self.y[-1])
    v_now_y = self.y[0] * np.sin(self.y[-1]) + self.y[1] * np.cos(self.y[-1])
    depths = []
    for i in range(MPC_prediction_horizon + 1):
        depths.append(cal_depth([self.y[3] + i * MPC_time_step * u_now_x, self.y[4] + i * MPC_time_step * v_now_y], self.depths_data))
    Am = self.ontology.B[0] * self.ontology.Td[0]
    beff = 4 * self.ontology.B[0]
    Ac = beff * depths[0]
    S_f = 0.98 * Am / Ac
    K_barrass = min(2, max(1, 5.74 * S_f**0.76))
    z0_calc = 0.01 * K_barrass * self.ontology.Cb[0] * ((self.y[0]**2 + self.y[1]**2)) * (3.6 / 1.852)**2
    yB3_current = 0.5 * self.ontology.B[0] * (1 / (-(0.5 * beff + myvessel.y[4])) + 1 / (0.5 * beff - myvessel.y[4]))
    XH, YH, NH, XP, XR, YR, NR, YB, NB, U = calculate_forces(self, v_sym, delta, Np, depth=depths[0], sinkage=x_3, yB3_k=yB3_current)
    h = np.min(depths)
    Ac = beff * h
    S_f = 0.98 * Am / Ac
    K_barrass = min(2, max(1, 5.74 * S_f**0.76))
    alg_fun = x_3 - 0.01 * K_barrass * self.ontology.Cb[0] * (U * 3.6 / 1.852)**2
    model.set_alg('x_3', alg_fun)
    model.set_rhs('x_4', x_7)
    XH = XH * self.ontology.force_switch[0]
    YH = YH * self.ontology.force_switch[0]
    NH = NH * self.ontology.force_switch[0]
    XP = XP * self.ontology.force_switch[1]
    XR = XR * self.ontology.force_switch[2]
    YR = YR * self.ontology.force_switch[2]
    NR = NR * self.ontology.force_switch[2]
    YB = YB * self.ontology.force_switch[3]
    NB = NB * self.ontology.force_switch[3]
    m11_nd, m22_nd, m26_nd, m66_nd = cal_addedmass_nd(hdratio=1.2)
    Td_affected = self.ontology.Td[0] + 0.01 * K_barrass * self.ontology.Cb[0] * (U * 3.6 / 1.852)**2
    m11 = m11_nd * (0.5 * 1025 * self.ontology.L[0]**2 * Td_affected)
    m22 = m22_nd * (0.5 * 1025 * self.ontology.L[0]**2 * Td_affected)
    m26 = m26_nd * (0.5 * 1025 * self.ontology.L[0]**3 * Td_affected)
    m66 = m66_nd * (0.5 * 1025 * self.ontology.L[0]**4 * Td_affected)
    X_d = (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * (XP + XR) + (1025 * 0.5 * self.ontology.L[0] * Td_affected * U**2) * XH
    Y_d = (1025 * 0.5 * self.ontology.L[0] * self.ontology.Td[0] * U**2) * (YR + YB) + (1025 * 0.5 * self.ontology.L[0] * Td_affected * U**2) * YH
    N_d = (1025 * 0.5 * self.ontology.L[0]**2 * self.ontology.Td[0] * U**2) * (NR + NB) + (1025 * 0.5 * self.ontology.L[0]**2 * Td_affected * U**2) * NH
    Y_m = Y_d - (self.ontology.m[0] + m11) * x_5 * x_7
    N = N_d - Y_d * self.ontology.xg[0]
    vdot_num_matrix_det = Y_m * (self.ontology.Iz[0] + m66) - N * m26
    rdot_num_matrix_det = (self.ontology.m[0] + m22) * N - m26 * Y_m
    coeff_matrix_det = (self.ontology.m[0] + m22) * (self.ontology.Iz[0] + m66) - m26 * m26
    model.set_rhs('x_5', (X_d + (self.ontology.m[0] + m22) * x_6 * x_7) / (self.ontology.m[0] + m11))
    model.set_rhs('x_6', vdot_num_matrix_det / coeff_matrix_det)
    model.set_rhs('x_7', rdot_num_matrix_det / coeff_matrix_det)
    model.setup()

    vesselmpc = mpc.controller.MPC(model)
    vesselmpc.set_param(
        n_horizon=MPC_prediction_horizon,
        t_step=MPC_time_step,
        store_full_solution=False,
        nlpsol_opts={"ipopt.linear_solver": "mumps", "ipopt.print_level": 0, "ipopt.sb": "yes", "print_time": 0}
    )
    tvp_template = vesselmpc.get_tvp_template()
    def tvp_fun(t_now):
        for k in range(vesselmpc.settings.n_horizon + 1):
            tvp_template['_tvp', k, 'psi_ref'] = psi_d
        return tvp_template
    vesselmpc.set_tvp_fun(tvp_fun)
    mterm = (1) * casadi.power((-x_4 + psi_ref), 2) + (-3e-6) * (casadi.power(x_5, 2) + casadi.power(x_6, 2))
    lterm = (1) * casadi.power((-x_4 + psi_ref), 2) + (-3e-6) * (casadi.power(x_5, 2) + casadi.power(x_6, 2))
    vesselmpc.set_objective(mterm=mterm, lterm=lterm)
    vesselmpc.set_rterm(Np=1e-7, delta=1e-7)
    vesselmpc.bounds['lower', '_u', 'Np'] = 100 / 60
    vesselmpc.bounds['upper', '_u', 'Np'] = 500 / 60
    vesselmpc.bounds['lower', '_u', 'delta'] = -35 * np.pi / 180
    vesselmpc.bounds['upper', '_u', 'delta'] = 35 * np.pi / 180
    U_max = 7 * 1.852 / 3.60
    nonlinear_expr = x_5**2 + x_6**2
    vesselmpc.set_nl_cons('cons', nonlinear_expr, U_max**2)
    UKC_min = 0.05
    UKC_buff = 0.15
    nonlinear_expr = x_3 + self.ontology.Td[0] + UKC_min + UKC_buff - np.min(depths)
    vesselmpc.set_nl_cons('cons_g', nonlinear_expr, 0, soft_constraint=True, penalty_term_cons=1e-3, maximum_violation=UKC_buff)
    vesselmpc.setup()
    vesselmpc.x0 = np.array(self.y[3:5] + self.y[5:] + self.y[0:3]).reshape(-1, 1)
    vesselmpc.z0 = z0_calc
    vesselmpc.u0 = np.array([self.Np[-1], self.u[-1] * np.pi / 180])
    vesselmpc.set_initial_guess()
    return vesselmpc

def get_next_mpc_step(vessel):
    x0 = np.array(vessel.state[3:5] + vessel.state[6:] + vessel.state[0:3]).reshape(-1, 1)
    u = vessel.MPC.make_step(x0)
    return u

def PID_controller(self, psi_d, dt):
    """
    Implements a PID (Proportional-Integral-Derivative) controller for a vessel.
    Returns the commanded rudder angle (delta) in radians, clipped to +-35 deg.
    """
    A = 4
    Yd = -3 * A / self.ontology.L[0]**2
    Nd = -Yd / 2
    xg_nd = self.ontology.xg[0] / self.ontology.L[0]
    Iz_nd = self.ontology.Iz[0] / (0.5 * 1025 * self.ontology.L[0]**5)
    m_nd = self.ontology.m[0] / (0.5 * 1025 * self.ontology.L[0]**3)
    Yvdot = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 + 0.16 * self.ontology.Cb[0] * self.ontology.B[0] / self.ontology.Td[0] - 5.1 * (self.ontology.B[0] / self.ontology.L[0])**2)
    Yrdot = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (0.67 * (self.ontology.B[0] / self.ontology.L[0]) - 0.0033 * (self.ontology.B[0] / self.ontology.Td[0])**2)
    Nvdot = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1.1 * (self.ontology.B[0] / self.ontology.L[0]) - 0.041 * (self.ontology.B[0] / self.ontology.Td[0]))
    Nrdot = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 / 12 + 0.017 * self.ontology.Cb[0] * (self.ontology.B[0] / self.ontology.Td[0]) - 0.33 * (self.ontology.B[0] / self.ontology.L[0]))
    Yv = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 + 0.4 * self.ontology.Cb[0] * (self.ontology.B[0] / self.ontology.Td[0]))
    Nv = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (0.5 + 2.4 * self.ontology.Td[0] / self.ontology.L[0])
    Nr = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (1 / 4 + 0.039 * (self.ontology.B[0] / self.ontology.Td[0]) - 0.56 * (self.ontology.B[0] / self.ontology.L[0]))
    Yr = -np.pi * (self.ontology.Td[0] / self.ontology.L[0])**2 * (-1 / 2 + 2.2 * (self.ontology.B[0] / self.ontology.L[0]) - 0.08 * (self.ontology.B[0] / self.ontology.Td[0]))

    K_nd = (Nv * Yd - Yv * Nd) / (Nr * Yv + (-Yr + m_nd) * Nv)
    T1_2 = ((Yvdot - m_nd) * (Nr - m_nd * xg_nd) + (Nrdot - Iz_nd) * Yv - (Yrdot - m_nd * xg_nd) * Nv - (Nvdot - m_nd * xg_nd) * (Yr - m_nd)) / (Yv * (Nr - m_nd * xg_nd) - Nv * (Yr - m_nd))
    T3 = ((Nv - m_nd * xg_nd) * Yd - (Yvdot - m_nd) * Nd) / (Nv * Yd - Yv * Nd)
    T_nd = T1_2 - T3
    U = 2.5
    T = T_nd * self.ontology.L[0] / U
    K = K_nd * U / self.ontology.L[0]

    wn = 0.1
    zeta = 0.95
    Kp = (T / K) * wn**2
    Kd = (T / K) * (2 * zeta * wn - 1 / T)
    Td = Kd / Kp
    Ti = 10 / wn

    # State variables (using vessel's seeded RNG for measurement noise)
    psi = self.ontology.ypsi[-1]
    rng = getattr(self, 'rng', np.random.default_rng(42))
    r = self.state[2] * (1.0 + self.measurement_noise * rng.uniform(-1.0, 1.0))
    error_psi = wrap_angle_to_pmpi(psi_d - psi)
    r_d = wrap_angle_to_pmpi(psi_d - self.psi_d_prev) / dt
    self.aggregated_error = self.aggregated_error + error_psi * dt

    # PID control law for rudder angle
    delta = Kp * (error_psi + (1 / Ti) * self.aggregated_error + Td * (-r + r_d))
    delta = np.clip(delta, -35 * np.pi / 180, 35 * np.pi / 180)
    return delta

def plan_route(start, stop):
    x_ref = np.linspace(start[0], stop[0], 20)
    y_ref = np.linspace(start[1], stop[1], 20)
    wp_pos = np.array([x_ref, y_ref])
    return wp_pos

def wrap_angle_to_pmpi(angle):
    wrapped_angle = (angle + np.pi) % (2 * np.pi) - np.pi
    return wrapped_angle

def check_for_wp_segment_switch(self, wp_segment, d_0wp):
    wp_segment = self.normalize_vec(wp_segment)
    d_0wp = self.normalize_vec(d_0wp)
    segment_passed = np.dot(wp_segment, d_0wp) < np.cos(np.deg2rad(self.pass_angle_threshold))
    return segment_passed

def normalize_vec(self, vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm