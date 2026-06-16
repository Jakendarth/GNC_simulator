import numpy as np
import csv

def get_depths_from_file(filename='assets/ChannDepths.csv'):
    depths_data=[]
    with open(filename, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.reader(infile)
        for row in reader:
            depths_data.append([float(row[0]),float(row[1])])
    depths_data=np.array(depths_data)
    return depths_data

def cal_depth(v,depths_data):
    """
        Calculate the channel depth of the current position
        Input: coordinates, v[0] the x coordinates; v[1] the y coordinates
        Input: depths_data (if input [] then use the local profile in the function)
        Output: depth (float)
        Rewritten to use .csv file as input
    """
    # a function or look-up operation to get the depth
    # x-range: -100 to 900
    if len(depths_data)==0: # no input depths_data (from file)
        Td_const=2.74
        if v[0]<-50:
            h=1.20*Td_const
        elif v[0]<200:
            h=1.15*Td_const
        elif v[0]<400:
            h=1.20*Td_const
        elif v[0]<550:
            h=1.25*Td_const
        elif v[0]<700:
            h=1.20*Td_const
        else:
            h=1.30*2.74
    else: # depths data input from file
        h=np.interp(v[0],depths_data[:,0],depths_data[:,1])    
    # h = 1.2*2.74 # 1.2*Td meters constant depth for testing
    return h

def cal_addedmass_nd(hdratio):
    """
        Calculate the added mass coefficients with h/d ratios (linearized around h/d=1.2)
        Input: hdratio (float), 1.0 to 2.0
        Output: m11_nd, m22_nd, m26_nd, m66_nd
    """
    m11_nd=(-2.772E-02)*(hdratio)+(5.277E-02)
    if m11_nd<1.154E-02:
        m11_nd=1.154E-02
    m22_nd=(-8.757E-01)*(hdratio)+(1.423E+00)
    if m22_nd<1.604E-01:
        m22_nd=1.604E-01
    m26_nd=(-4.640E-02)*(hdratio)+(6.648E-02)
    if m26_nd<2.629E-03:
        m26_nd=2.629E-03
    m66_nd=(-2.241E-02)*(hdratio)+(3.929E-02)
    if m66_nd<6.891E-03:
        m66_nd=6.891E-03
    return m11_nd,m22_nd,m26_nd,m66_nd

def retrieve_t_from_x(t_vals,x_vals,x_sample):
    """"
    get the time when travelling through the point x_sample (with the nearest value in t_vals)
    input: data set t_vals, x_vals, assuming that x_vals always increasing (if not, the first time)
    input: x_sample
    """
    t_vals=np.array(t_vals)
    if x_sample < np.min(x_vals) or x_sample > np.max (x_vals): # check validity
        print(f"x_sample={x_sample:.3f} not valid")
        return 0, t_vals[0]
    t_interp=np.interp(x_sample,x_vals,t_vals)
    t_index=np.abs(t_vals-t_interp).argmin()
    return t_index, t_vals[t_index]

def calc_UKC_violations(t_vals,UKC_vals,UKC_limit=0.20):
    """
    calculate the violations when UKC<UKC_limit
    input: t_vals, UKC_vals, UKC_limit (defaut 0.20m)
    output: pair of violations, time and index
    """
    t_violations=[]
    index_t_violations=[]
    flag_violation=0
    t_start=0
    i_start=0
    for i in range(len(t_vals)):
        t=t_vals[i]
        if UKC_vals[i]<UKC_limit and flag_violation==0:
            t_start=t
            i_start=i
            flag_violation=1
        elif UKC_vals[i]>=UKC_limit and flag_violation==1:
            t_violations.append([t_start,t])
            index_t_violations.append([i_start,i])
            flag_violation=0
    if flag_violation==1:
        t_violations.append([t_start,t_vals[-1]])
        index_t_violations.append([i_start,len(t_vals)-1])
    return t_violations,index_t_violations