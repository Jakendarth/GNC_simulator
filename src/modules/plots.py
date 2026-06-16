import matplotlib.animation as animation
from matplotlib import rc
from matplotlib import ticker
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches
import datetime
from itertools import cycle
from scipy.interpolate import interp1d
import pygame
from channel import *
#from mpl_toolkits.basemap import Basemap
#import geopandas as gpd


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
    
    print(self.shapefile.head())  # Display the first few rows of the data
    print(self.shapefile.columns)  # Display the column names
    print(self.shapefile.crs)  # Display the coordinate reference system
    print(self.shapefile.geometry)  # Display the geometry column
    print(self.shapefile['vrt_naam'])

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
        print(points)
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
    ax.plot(x, y, linewidth=1, color='red',linestyle='--')
    ax.plot(outer_line1_x,outer_line1_y, linewidth=1, color='orange')
    ax.plot(outer_line2_x,outer_line2_y, linewidth=1, color='orange')
    ax.fill_between(outer_line2_x, outer_line2_y, ax.get_ylim()[0], color='orange')
    ax.fill_between(outer_line2_x, outer_line2_y, outer_line1_y, color='b')
    ax.fill_between(outer_line1_x, ax.get_ylim()[1], outer_line1_y,  color='orange')
    ax.set_title('Zandkreek en Veerse Meer')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.show()

def plotvessel(t, y, start_at_frame = 0, interval = 100, \
                 ani_name = "vessel_movement.gif"):

    frames = len(t)-start_at_frame

    # We make a figure with two subplots
    fig = plt.figure(figsize=(10,5))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    # We add the first data points to the (x,y) plot and 
    # set the limits
    '''
    ax1.set_xlim(0, 10)
    ax1.set_ylim(0, 10)
    ax1.set_xticks([0,2,4,6,8,10])
    ax1.set_yticks([0,2,4,6,8,10])
    '''
    ax1.set_xlabel("x (m)")
    ax1.set_ylabel("y (m)")
    ax1.grid('minor')
    # We add a dot and line to the (x,y) plot
    dot1, = ax1.plot(0, 0, "ko")
    '''
    ax2.set_xlim(0, 1)
    ax2.set_xticks([0,0.5,1])
    ax2.set_ylim(-1, 1)
    ax2.set_yticks([-1,0,1])
    '''
    ax2.set_xlabel("u (m/s)")
    ax2.set_ylabel("v (m/s)")
    ax2.grid('minor')
    # We add a dot and line to the (x,y) plot
    dot2, = ax2.plot(0.364, 0, "ko")
    # We make the second plot showing theta as a 
    # function of time
    #ax2.plot(t, y[5], "gray")
    #ax2.axhline(0, color='r', linestyle='-')
    # We also add a dot for the animation
    #dot2, = ax2.plot(0, 0, "ko")
   # ax2.set_xlabel("t (sec)")
    #ax2.set_ylabel("$\\psi$ (radians)")
    #ax2.yaxis.set_label_position("right")
    #ax2.yaxis.tick_right()
    
    # This function updates the plot for the animation
    def update(frame):
        # Updating the plots
        xi, yi = y[3],y[4]
        dot1.set_data([xi], [yi])
        xi, yi = y[1],y[2]
        dot2.set_data([xi], [yi])
        #line.set_data([0, xi], [0, yi])
        #dot2.set_data([t[frame]], [y[frame]])

    # Create the animation
    ani = animation.FuncAnimation(fig=fig, func=update,frames=frames, interval=interval)
    ani.save(ani_name)
    #plt.show()
    plt.close()

    return ani 

def encounter_scenario(nr_crossings,nr_headon,nr_overtaking,sim_time, start_cond):
    nr_vessels=nr_crossings+nr_headon+nr_overtaking-1
    time_events=np.random.uniform(0,sim_time,[1,nr_vessels+1])
    head_on=['HO' for ii in range(nr_headon+1)]
    crossing=['CR' for ii in range(nr_crossings+1)]
    overtaking=['OT' for ii in range(nr_overtaking+1)]
    scenario=head_on+crossing+overtaking
    scenario=np.random.shuffle(scenario)
    print(scenario)
    print(time_events)

def draw_vessel(vessel,x,y,psi,t, target_time):
    """
        Computes the transformed coordinates of a vessel's outline at a specific target time.

        Parameters:
            vessel (object): An object representing the vessel, which contains ontology attributes
                             such as `B` (beam) and `L` (length).
            x (list or numpy.ndarray): The x-coordinates of the vessel's position over time.
            y (list or numpy.ndarray): The y-coordinates of the vessel's position over time.
            psi (list or numpy.ndarray): The heading angles (in radians) of the vessel over time.
            t (list or numpy.ndarray): The time points corresponding to the vessel's position and heading.
            target_time (float): The specific time at which the vessel's outline is to be computed.

        Returns:
            tuple: A tuple containing two numpy arrays:
                - new_xpos: The transformed x-coordinates of the vessel's outline at the target time.
                - new_ypos: The transformed y-coordinates of the vessel's outline at the target time.

        Notes:
            - The function determines the closest time index to `target_time` in the `t` array.
            - The vessel's outline is approximated as an ellipse, with the beam (`B`) and length (`L`)
              defining its dimensions.
            - The outline is rotated and translated based on the vessel's heading (`psi`) and position
              (`x`, `y`) at the determined time index.
    """
    index=np.argmin(np.array([np.abs(t1-target_time) for t1 in t]),axis=0)
    print('Target time:',target_time)
    #print(np.array([t1-target_time for t1 in t]))
    print('Index:',index)
    theta=np.linspace(0,2*np.pi,100)
    ypos = vessel.ontology.B[0]/2*np.sin(theta)
    xpos= vessel.ontology.L[0]/2*np.cos(theta)
    new_xpos = xpos*np.cos(psi[index])-ypos*np.sin(psi[index])+x[index]
    new_ypos = xpos*np.sin(psi[index])+ypos*np.cos(psi[index])+y[index]
    return new_xpos, new_ypos

def plots(x_vals,y_vals,psi_vals,z_vals,U_vals,times,d1,d2,myvessel,total_time, step):
    ref_route=myvessel[0].route
    droute=myvessel[0].droute
    # Movement of the own vessel compared to target vessels
    fig,ax=plt.subplots(1, 1, figsize=(10, 10))
    plt.plot(x_vals[0], y_vals[0],'b-',label='Own vessel')
    plt.plot(x_vals[1], y_vals[1],color='green', linestyle='-',label='Target vessel 1')
    plt.plot(x_vals[2], y_vals[2],color='magenta', linestyle='-',label='Target vessel 2')
    stb_bank = patches.Rectangle((400, 200), 600, 200, linewidth=1, edgecolor='k', facecolor='k')
    port_bank= patches.Rectangle((400, -400), 600, 200, linewidth=1, edgecolor='k', facecolor='k')
    theta=np.linspace(0,2*np.pi,100)
    for target_time in [0,150,320,520]:#,120,160,200,240,280,320,360]:
        x1,y1=draw_vessel(myvessel[0],x_vals[0],y_vals[0],psi_vals[0],times[0],target_time)
        plt.plot(x1,y1,'b')
        if target_time==0:
            index=np.argmin(np.array([np.abs(t1-target_time) for t1 in times[0]]),axis=0)
            plt.plot(x_vals[0][index]+0.7*myvessel[0].ontology.L[0]*np.cos(theta),y_vals[0][index]+0.7*myvessel[0].ontology.L[0]*np.sin(theta),color='orange',linestyle="-.", label='Safe distance')
        elif target_time==320:
            index=np.argmin(np.array([np.abs(t1-target_time) for t1 in times[0]]),axis=0)
            plt.plot(x_vals[0][index]+0.6*myvessel[0].ontology.L[0]*np.cos(theta),y_vals[0][index]+0.6*myvessel[0].ontology.L[0]*np.sin(theta),color='orange',linestyle="-.")
    for target_time in [0,150,320]:#,120,160,200,240,280,320,360]:
        x2,y2=draw_vessel(myvessel[1],x_vals[1],y_vals[1],psi_vals[1],times[1],target_time)
        plt.plot(x2,y2,'g')
        if target_time==150:
            index=np.argmin(np.array([np.abs(t1-target_time) for t1 in times[1]]),axis=0)
            plt.plot(x_vals[1][index]+0.7*myvessel[1].ontology.L[0]*np.cos(theta),y_vals[1][index]+0.7*myvessel[1].ontology.L[0]*np.sin(theta),color='orange',linestyle="-.")
    for target_time in [520,670]:#,120,160,200,240,280,320,360]:
        x3,y3=draw_vessel(myvessel[2],x_vals[2],y_vals[2],psi_vals[2],times[2],target_time)
        plt.plot(x3,y3,'m')
        if target_time==520:
            index=np.argmin(np.array([np.abs(t1-target_time) for t1 in times[2]]),axis=0)
            plt.plot(x_vals[2][index]+0.6*myvessel[2].ontology.L[0]*np.cos(theta),y_vals[2][index]+0.6*myvessel[2].ontology.L[0]*np.sin(theta),color='orange',linestyle="-.")
    #ax[0, 0].plot(ref_route[0]+droute[0],ref_route[1]+droute[1],'m--')
    ax.add_patch(stb_bank)
    ax.add_patch(port_bank)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid('minor')
    ax.set_xlim(-220, 1000)
    ax.set_ylim(-400, 400)
    now = datetime.datetime.now()
    now_str = now.strftime("%Y-%m-%d-%H-%M-%S")
    plt.savefig('output/Navigation_'+now_str+'.png')
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(times[0], U_vals,'b')
    ax.set_xlabel('Time [sec]')
    ax.set_ylabel('U [deg]')
    ax.grid('minor')
    plt.savefig('output/vessel_speed.png')
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    #ax[0, 1].plot(myvessel.times, [chi*180/np.pi for chi in myvessel.chi],'m--')
    ax.plot(times[0], [psi*180/np.pi for psi in psi_vals[0]],'b')
    ax.set_xlabel('Time [sec]')
    ax.set_ylabel(r'$\psi$ [deg]')
    ax.grid('minor')
    plt.savefig('output/heading_angle.png')
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(times[0], d1,'k',label='Port side bank distance')
    ax.plot(times[0], d2,'r',label='Starboard side bank distance')
    ax.legend()
    ax.set_ylim(150, 250)
    ax.set_xlabel('Time [sec]')
    ax.set_ylabel('Distance [m]')
    ax.grid('minor')   
    plt.savefig('output/bank_distance.png')
    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    a_interp_at_t2 = np.interp(x_vals[0],ref_route[0]+droute[0],ref_route[1]+droute[1])
    ax.plot(times[0], [(-a_i + b_i)/myvessel[0].ontology.L[0] for a_i, b_i in zip(a_interp_at_t2.tolist(), y_vals[0])],'b')
    #ax[1, 1].plot(times[0],[myvessel[0].ontology.Td[0]*0.2 for t in times],'r--',label='Grounding limit')
    #ax[1, 1].legend()
    ax.set_xlabel('Time [sec]')
    ax.set_ylabel('Normalised referece trajectory tracking error [L]')
    #ax[1, 1].legend()
    ax.grid('minor')
    now = datetime.datetime.now()
    now_str = now.strftime("%Y-%m-%d-%H-%M-%S")
    plt.savefig('output/tracking_error.png')
    #times1 = np.arange(0, total_time, step)
    #times1 = [times[0][ii] for ii in range(0,len(times[0])) if x_vals[0][ii]>=400]
    #times1= np.arange(times1[0],times1[-1]+0.2,0.2)
    plt.figure()
    plt.plot(myvessel[0].ratio_time,myvessel[0].ratio_head_on,label='Actual',color='blue',linestyle='-')
    plt.plot(myvessel[0].ratio_time,myvessel[0].head_on_upperbound,label='Upper bound',color='magenta',linestyle='--')
    plt.plot(myvessel[0].ratio_time,myvessel[0].head_on_lowerbound,label='Lower bound',color='green',linestyle='--')
    plt.ylabel(r'$\frac{y_{cc}}{d_l}$ [-]')
    plt.xlabel('Time [sec]')
    plt.yticks([0,0.2,0.6,0.8])
    plt.xticks(np.arange(300,400,20))
    plt.xlim(300,400)
    plt.ylim(0,.8)
    plt.grid('minor')
    plt.legend()
    plt.savefig('output/Head_on_'+now_str+'.png')

def plot_FSM():
        print('Finalising FSM plot...')
        '''
        angle_domain = np.linspace(0, 2*np.pi, 2000).tolist()
        psi_h=6*np.pi/180
        D_hdn=[]
        D_pcr=[]
        D_scr=[]
        D_ovi=[]
        D_ove=[]
        D_safe=[]
        for psi_b in angle_domain:
            for psi_c in angle_domain:
                psi_c=psi_c%(2*np.pi)
                psi_b=psi_b%(2*np.pi)
                if ((psi_c>np.pi/2) and (psi_b<3*np.pi/2) and (abs(psi_b-psi_c)<np.pi/2)):
                    D_safe.append([psi_c,psi_b])
                else:
                    if ((psi_c>=(np.pi-psi_h)) and (psi_c<(np.pi+psi_h))):
                        D_hdn.append([psi_c,psi_b])
                    elif ((psi_c>=(np.pi+psi_h)) and (psi_c<13*np.pi/8)):
                        D_scr.append([psi_c,psi_b])
                    elif ((psi_c>=3*np.pi/8) and (psi_c<(np.pi-psi_h))):
                        D_pcr.append([psi_c,psi_b])
                    else:
                        if ((psi_b>=5*np.pi/8) and (psi_b<11*np.pi/8)):
                            D_ovi.append([psi_c,psi_b])
                        elif ((((np.pi+psi_b-psi_c)%(2*np.pi))>=5*np.pi/8) and (((np.pi+psi_b-psi_c)%(2*np.pi))<11*np.pi/8)):
                            D_ove.append([psi_c,psi_b])
                        elif (psi_b<np.pi):
                            D_scr.append([psi_c,psi_b]) 
                        else:
                            D_pcr.append([psi_c,psi_b])
        fig,ax=plt.subplots(figsize=(10, 8))
        if D_hdn:
            D_hdn = np.array(D_hdn)
            ax.scatter(D_hdn[:, 0], D_hdn[:, 1], color='yellow', label='Head On')
        if D_safe:
            D_safe = np.array(D_safe)
            ax.scatter(D_safe[:, 0], D_safe[:, 1], color='blue', label='Safe')
        if D_ovi:
            D_ovi = np.array(D_ovi)
            ax.scatter(D_ovi[:, 0], D_ovi[:, 1], color='black', label='Overtaking')
        if D_ove:
            D_ove = np.array(D_ove)
            ax.scatter(D_ove[:, 0], D_ove[:, 1], color='darkgrey', label='Overtaken')
        if D_scr:
            D_scr = np.array(D_scr)
            ax.scatter(D_scr[:, 0], D_scr[:, 1], color='red', label='Starboard crossing')
        if D_pcr:
            D_pcr = np.array(D_pcr)
            ax.scatter(D_pcr[:, 0], D_pcr[:, 1], color='green', label='Port crossing')
        
        plt.xlabel(r'$\psi_c$')
        plt.ylabel(r'$\psi_b$')
        plt.xlim(0, 2*np.pi)
        plt.ylim(0, 2*np.pi)
        ax.set_xticks(np.arange(0, 2*np.pi+0.01, np.pi/3))
        ax.set_yticks(np.arange(0, 2*np.pi+0.01, np.pi/3))
        labels = ['$0$', r'$\pi/3$', r'$2\pi/3$', r'$\pi$', r'$4\pi/3$',
            r'$5\pi/3$', r'$2\pi$']
        ax.set_xticklabels(labels)
        ax.set_yticklabels(labels)
        plt.legend()
        plt.title('Vessel encounter scenario classification')
        plt.grid('minor')
        plt.savefig('output/FSM.png')
        '''
        w=40
        B=10.67
        d_lc0=0.5*w
        d_lc_end=0.3*w
        d_lc=np.linspace(d_lc0,d_lc_end,2000).tolist()
        lv=100.96
        ratio=[d_lc0/d_lc[ii]-(lv*np.tan(-4*np.pi/180))/d_lc[ii]-1 for ii in range(0,len(d_lc))]
        d_lcs=np.linspace(0.2*w, 0.5*w, 2000).tolist()
        upperbound=[(0.8*w/d_lcs[ii])-1 for ii in range(0,len(d_lcs))]
        upperbound2=[0.4*w for ii in range(0,len(d_lcs))]
        lowerbound=[(0.5*B)/(0.8*w-0.5*B) for ii in range(0,len(d_lcs))]
        lowerbound2=[(0.5*B)/(d_lcs[ii]) for ii in range(0,len(d_lcs))]
        plt.figure()
        #plt.plot(d_lcs,upperbound2,label='Upper bound 2',color='red')
        plt.plot(d_lc,ratio,label='Actual',color='blue')
        plt.plot(d_lcs,upperbound,label='Upper bound',color='magenta',linestyle='--')
        #plt.plot(d_lcs,lowerbound,label='Lower bound',color='blue')
        plt.plot(d_lcs,lowerbound2,label='Lower bound',color='green',linestyle='--')
        plt.ylabel(r'$\frac{y_{cc}}{d_l}$ [-]')
        plt.xlabel(r'$d_l$ [m]')
        plt.legend()
        plt.show()

def plot_reg_switching(ontology,step,total_time):
    times = np.arange(0, total_time, step)
    fig, ax = plt.subplots(3, 3, figsize=(50, 25))
    ax[0][0].set_axis_off()
    ax[0][2].set_axis_off()
    regulations_class = ontology.search_one(iri="*Regulations")
    lines1 = ["-","--","-.",":"]
    lines2=lines1
    lines3=lines2
    lines4=lines3
    lines5=lines4
    lines6=lines5
    lines7=lines6
    linecycler1 = cycle(lines1)
    linecycler2 = cycle(lines2)
    linecycler3 = cycle(lines3)
    linecycler4 = cycle(lines4)
    linecycler5 = cycle(lines5)
    linecycler6 = cycle(lines6)
    linecycler7 = cycle(lines7)
    plt.rcParams["font.family"] = "Times New Roman"
    plt.rcParams["font.size"] = 24
    for individual in list(regulations_class.instances()):
        #individual.reg_active=individual.reg_active[1:]
        if ("Article_630" in individual.name) or ("Rule_6" in individual.name):
            ax[0,1].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler1))
            ax[0,1].set_xlabel('Time [sec]', size=24)
            ax[0,1].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[0,1].set_yticks([0,0.5,1])
            ax[0,1].tick_params(axis="x", labelsize=20)
            ax[0,1].tick_params(axis="y", labelsize=20)
            ax[0,1].grid('minor')
            ax[0,1].set_title("(a)", size=24)
            ax[0,1].legend()
        elif ("Article_609" in individual.name) or ("Article_610" in individual.name) or ("Rule_13" in individual.name):
            ax[1,0].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler2))
            ax[1,0].set_xlabel('Time [sec]', size=24)
            ax[1,0].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[1,0].set_yticks([0,0.5,1])
            ax[1,0].tick_params(axis="x", labelsize=20)
            ax[1,0].tick_params(axis="y", labelsize=20)
            ax[1,0].grid('minor')
            ax[1,0].set_title("(b)", size=24)
            ax[1,0].legend()
        elif ("Article_605" in individual.name) or ("Article_604" in individual.name)  or ("Rule_14" in individual.name):
            ax[1,1].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler3))
            ax[1,1].set_xlabel('Time [sec]', size=24)
            ax[1,1].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[1,1].set_yticks([0,0.5,1])
            ax[1,1].tick_params(axis="x", labelsize=20)
            ax[1,1].tick_params(axis="y", labelsize=20)
            ax[1,1].grid('minor')
            ax[1,1].set_title("(c)", size=24)
            ax[1,1].legend()
        elif ("Article_617" in individual.name) or ("Rule_15" in individual.name):
            ax[1,2].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler4))
            ax[1,2].set_xlabel('Time [sec]', size=24)
            ax[1,2].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[1,2].set_yticks([0,0.5,1])
            ax[1,2].tick_params(axis="x", labelsize=20)
            ax[1,2].tick_params(axis="y", labelsize=20)
            ax[1,2].grid('minor')
            ax[1,2].set_title("(d)", size=24)
            ax[1,2].legend()
        elif ("Article_603" in individual.name) or ("Rule_16" in individual.name):
            ax[2,0].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler5))
            ax[2,0].set_xlabel('Time [sec]', size=24)
            ax[2,0].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[2,0].set_yticks([0,0.5,1])
            ax[2,0].tick_params(axis="x", labelsize=20)
            ax[2,0].tick_params(axis="y", labelsize=20)
            ax[2,0].grid('minor')
            ax[2,0].set_title("(e)", size=24)
            ax[2,0].legend()
        elif ("Article_609" in individual.name) or ("Rule_17" in individual.name):
            ax[2,1].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler6))
            ax[2,1].set_xlabel('Time [sec]', size=24)
            ax[2,1].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[2,1].set_yticks([0,0.5,1])
            ax[2,1].tick_params(axis="x", labelsize=20)
            ax[2,1].tick_params(axis="y", labelsize=20)
            ax[2,1].grid('minor')
            ax[2,1].set_title("(f)", size=24)
            ax[2,1].legend()
        elif ("Article_629" in individual.name) or ("Rule_19" in individual.name):
            ax[2,2].plot(times,individual.reg_active, label=individual.name,linestyle=next(linecycler7))
            ax[2,2].set_xlabel('Time [sec]', size=24)
            ax[2,2].set_ylabel(r'$\mathcal{F}_r$', size=24)
            ax[2,2].set_yticks([0,0.5,1])
            ax[2,2].tick_params(axis="x", labelsize=20)
            ax[2,2].tick_params(axis="y", labelsize=20)
            ax[2,2].grid('minor')
            ax[2,2].set_title("(g)", size=24)
            ax[2,2].legend()
    now = datetime.datetime.now()
    now_str = now.strftime("%Y-%m-%d-%H-%M-%S")
    plt.savefig('output/Reg_switching_'+now_str+'.png')

def plots_grounding(myvessel,suppress_output):
    # === Parameters ===
    UKC0=0.05 # minimum UKC required
    UKC1=0.15 # buffer UKC kept
    W_channel=4*10.67 # channel width
    # === Data preparation ===
    x_ref_route=myvessel.route[0]
    y_ref_route=myvessel.route[1]
    # x_range=[np.min(x_ref_route),np.max(x_ref_route)]
    # x_grid=np.linspace(x_range[0],x_range[1],num=5000)
    depths_data=get_depths_from_file(filename='assets/ChannDepths.csv')
    # depth_samples=[cal_depth([x_cor,0],depths_data) for x_cor in x_grid]
    psi_error=np.array(myvessel.ontology.ypsi) - np.array(myvessel.ontology.psi_d)
    # === KPI calculation ===
    y_deviation=myvessel.ontology.yy-np.interp(myvessel.ontology.x,x_ref_route,y_ref_route)
    max_tracking_err=np.max(np.abs(myvessel.ontology.yy-np.interp(myvessel.ontology.x,x_ref_route,y_ref_route))) # in meters
    max_abs_psi_err=np.max(np.abs(psi_error)) # in rads
    UKC=np.array(myvessel.ontology.UKC)
    min_UKC=np.min(UKC) # in meters
    mean_UKC=np.average(UKC) # in meters
    print(f"Max deviation to ref: {max_tracking_err:.2f} meters")
    print(f"Max yaw angle error: {max_abs_psi_err: .3E} rads")
    print(f"Mean UKC:             {mean_UKC:.3f} meters")
    print(f"Minimum UKC:          {min_UKC:.3f} meters")
    t_UKC_buffer_violations,index_UKC_buffer_violations=calc_UKC_violations(myvessel.ontology.time,myvessel.ontology.UKC,UKC_limit=UKC0+UKC1)
    UKC_buffer_violation_time=0.0 # in seconds
    try:
        for t_violation in t_UKC_buffer_violations:
            UKC_buffer_violation_time=UKC_buffer_violation_time+t_violation[1]-t_violation[0]
    except:
        pass
    print(f"Total UKC buff time:  {UKC_buffer_violation_time:.1f} seconds")
    ATE=[] # along-track error, with measurements
    CTE=[] # cross-track error, with measurements
    for i in range(len(myvessel.ontology.yx)):
        try:
            route_index=(np.array(x_ref_route)>myvessel.ontology.yx[i]).tolist().index(True) # index of the next waypoint
            route_index=route_index-1 # index of the last waypoint
            x_route_index=x_ref_route[route_index]
            y_route_index=y_ref_route[route_index]
            psi_route_index=0
            if(route_index>0):
                psi_route_index=np.atan2(y_ref_route[route_index+1]-y_route_index,x_ref_route[route_index+1]-x_route_index)
        except: # already pass all waypoints
            # route_index=len(x_ref_route)-1
            psi_route_index=0
            x_route_index=x_ref_route[-1]
            y_route_index=y_ref_route[-1]
        ATE.append((myvessel.ontology.yx[i]-x_route_index)*np.cos(psi_route_index)+(myvessel.ontology.yy[i]-y_route_index)*np.sin(psi_route_index))
        CTE.append(-(myvessel.ontology.yx[i]-x_route_index)*np.sin(psi_route_index)+(myvessel.ontology.yy[i]-y_route_index)*np.cos(psi_route_index))
    UKCA=np.array(myvessel.ontology.UKC)/(UKC0+UKC1) # UKC allowance percentage
    UKCA_mean=np.average(UKCA)
    UKCA_std=np.std(UKCA)
    # === Plots generation ===
    px = 1/plt.rcParams['figure.dpi']
    plt.rcParams['font.family'] = 'Arial' 
    plt.rcParams['font.size'] = 12
    figure, axs=plt.subplots(2,2,figsize=(1200*px, 700*px))
    axs=[axs[0,0],axs[0,1],axs[1,0],axs[1,1]]
    axs[0].plot(myvessel.ontology.time,myvessel.ontology.SOG,'b')
    axs[0].set_title(r'(a) Ship speed ($U$)')
    axs[0].set_ylabel(r'$U$ (m/s)')
    axs[0].set_xlabel('Time (s)')
    # axs[1,0].plot(x_grid,depth_samples)
    # axs[1,0].set_title('Depth profile')
    line_np=axs[1].plot(myvessel.ontology.time,[nP* 60 for nP in myvessel.ontology.propeller_speed],'b')
    line_np[0].set_label('nP')
    axs[1].set_title(r'(b) Propeller speed ($n_P$)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel(r'$n_P$ (RPM)')
    # axs[1].legend()
    line_delta=axs[2].plot(myvessel.ontology.time,[np.rad2deg(delta) for delta in myvessel.ontology.rudder_angle],'b',alpha=0.7)
    axs[2].set_title(r'(c) Rudder angle ($\delta$)')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel(r'$\delta$ (degree)')
    line_delta[0].set_label('delta')
    axs[3].set_title(r'(d) Rudder angle ($\delta$), $t=14\sim45$s')
    try:
        index_start=myvessel.ontology.time.index(14)
        index_end=myvessel.ontology.time.index(45)
        line_delta_sel=axs[3].plot(myvessel.ontology.time[index_start:index_end+1],[np.rad2deg(delta) for delta in myvessel.ontology.rudder_angle[index_start:index_end+1]],'b')
    except:
        pass
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel(r'$\delta$ (degree)')
    # axs[2].legend()
    figure.tight_layout()
    figure, axs=plt.subplots(1,2,figsize=(1200*px, 400*px))
    lines_psi=[]
    lines_psi.extend(axs[0].plot(myvessel.ontology.time,np.rad2deg(myvessel.ontology.psi_d),"#ffae00"))
    lines_psi.extend(axs[0].plot(myvessel.ontology.time,np.rad2deg(myvessel.ontology.ypsi),'b',alpha=0.50))
    lines_psi[0].set_label('Reference')
    lines_psi[1].set_label('Yaw angle')
    axs[0].set_title('(a) Heading angle')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel(r'Yaw angle ($^\circ$)')
    axs[0].legend()
    lines_psi_err=axs[1].plot(myvessel.ontology.time,np.rad2deg(psi_error),'b')
    axs[1].annotate(f"Max abs yaw angle\nerror:{np.rad2deg(max_abs_psi_err): .3f} degs",(0,(0+2*np.rad2deg(np.min(psi_error))/3)))
    axs[1].set_title('(b) Yaw angle error')
    lines_psi_err[0].set_label('Error')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel(r'Yaw angle ($^\circ$)')
    # axs[1].legend()
    figure.tight_layout()
    figure, axs=plt.subplots(1,2,figsize=(1200*px, 400*px))
    line_TD=axs[0].plot(myvessel.ontology.time,y_deviation,'b')
    axs[0].set_title('(a) Tracking deviation (TD)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('TD (m)')
    axs[0].annotate(f"Max.abs: {max_tracking_err:.4f}\nMean:    {np.mean(y_deviation):.4f}\nStd.dev: {np.std(y_deviation):.4f}",(0.2*myvessel.ontology.time[-1],-0.004))
    line_cte=axs[1].plot(myvessel.ontology.time,CTE,'b',alpha=0.7)
    axs[1].set_title('(b) Cross-track error (CTE)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('CTE (m)')
    figure.tight_layout()
    figure, axs=plt.subplots(1,2,figsize=(1200*px, 400*px))
    line_z=axs[0].plot(myvessel.ontology.time,myvessel.ontology.zhat,'b')
    axs[0].set_title(r'(a) Squat sinkage ($z$)')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Sinkage (m)')
    line_UKCA=axs[1].plot([myvessel.ontology.time[0],myvessel.ontology.time[-1]],[100,100],'k--',myvessel.ontology.time,UKCA*100,'b')
    axs[1].set_title('(b) UKC allowance (UKCA)')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('UKCA (%)')
    axs[1].yaxis.set_major_formatter(ticker.PercentFormatter(decimals=1))
    axs[1].annotate(f"Mean:    {100*UKCA_mean:.1f}%\nStd.dev: {100*UKCA_std:.1f}%",(0.2*myvessel.ontology.time[-1],40))
    figure.tight_layout()
    figure, axs=plt.subplots(2,1,figsize=(1200*px, 600*px))
    lines=axs[0].plot(x_ref_route,y_ref_route,'k',x_ref_route,y_ref_route,'ro',markerfacecolor='none')
    lines.extend(axs[0].plot(myvessel.ontology.x,myvessel.ontology.yy,'b'))
    lines[0].set_label('Reference')
    lines[1].set_label('Waypoints')
    lines[2].set_label('Trajectory')
    axs[0].set_title('(a) Path following')
    axs[0].set_xlabel('x (m)')
    axs[0].set_ylabel('y (m)')
    axs[0].set_ylim(-W_channel/2,W_channel/2)
    axs[0].invert_yaxis()
    axs[0].annotate(f"Max abs deviation: {max_tracking_err:.3f} meters",((x_ref_route[0]*2+x_ref_route[-1]*3)/5,(y_ref_route[0]+y_ref_route[-1]+W_channel)/4))
    axs[0].legend(loc='upper right')
    depth_values=[cal_depth([x_cor,0],depths_data) for x_cor in myvessel.ontology.x]
    channel_UKC_min=[depth-UKC0 for depth in depth_values]
    channel_UKC_buff=[depth-UKC0-UKC1 for depth in depth_values]
    vessel_keel=[z+myvessel.ontology.Td[0] for z in myvessel.ontology.zhat]
    vessel_draft=[myvessel.ontology.Td[0]]*len(myvessel.ontology.zhat)
    axs[1].set_title('(b) Grounding avoidance')
    axs[1].invert_yaxis()
    lines=axs[1].plot(myvessel.ontology.x,depth_values,'k',myvessel.ontology.x,vessel_keel,"#0022ff",myvessel.ontology.x,vessel_draft,'k--')
    area0=axs[1].fill_between(myvessel.ontology.x,depth_values,channel_UKC_min,color="#d00404")
    area1=axs[1].fill_between(myvessel.ontology.x,channel_UKC_min,channel_UKC_buff,color="#ffff009d")
    area2=axs[1].fill_between(myvessel.ontology.x,channel_UKC_buff,vessel_keel,color="#85ffff")
    area3=axs[1].fill_between(myvessel.ontology.x,vessel_keel,vessel_draft,color="#009519")
    try:
        for index in index_UKC_buffer_violations:
            axs[1].plot(myvessel.ontology.x[index[0]:index[1]],vessel_keel[index[0]:index[1]],'r')
    except:
        pass
    lines[0].set_label('Waterway floor')
    lines[1].set_label('Keel position')
    lines[2].set_label('Static draft')
    area0.set_label('Minimum UKC')
    area1.set_label('Buffer UKC')
    area2.set_label('Extra UKC')
    area3.set_label('Squat sinkage')
    axs[1].legend(handles=[lines[2], area3, lines[1], area2, area1, area0, lines[0]],loc='upper right')
    axs[1].set_xlabel('x (m)')
    axs[1].set_ylabel('Depth from water surface (m)')
    axs[1].annotate(f"Mean UKC: {mean_UKC:.3f} meters",((2*myvessel.ontology.x[0]+3*myvessel.ontology.x[-1])/5,myvessel.ontology.Td[0]+myvessel.ontology.zhat[-1]/4))
    axs[1].annotate(f"Min    UKC: {min_UKC:.3f} meters",((2*myvessel.ontology.x[0]+3*myvessel.ontology.x[-1])/5,myvessel.ontology.Td[0]+myvessel.ontology.zhat[-1]/4+0.05))
    figure.tight_layout()
    if (suppress_output==False):
        plt.show()

def plot_detailed_snapshots(d, t_step,time,t,vessels,X_vals,dbank1,dbank2,dbanko1,dbanko2,psi):
    """
    Creates a two-panel plot showing detailed snapshots of two separate encounters.
    Includes the path envelope, vessel positions, and a visualization of the optimization constraints.
    """
    #fig, axes = plt.subplots(1, 2, figsize=(20, 9), sharex=True, sharey=True)
    times = np.arange(0, time, t_step)
    pos = next(x[0] for x in enumerate(X_vals[0]) if x[1] > 600/0.2)
    print(pos)
    time_br=t[0][pos]
    pos2=next(x[0] for x in enumerate(times) if x[1] > time_br)
    pos2=pos2
    plt.figure()
    plt.plot(times[:pos2],np.array(d[0][:pos2])-np.ones(len(times[:pos2]))*vessels[0].ontology.B[0]/2,color=b_blue,label=r'$d_{0,1}- \rho_s$')
    plt.plot(times[pos2:],np.array(d[1][pos2:])-np.ones(len(times[pos2:]))*vessels[0].ontology.B[0]/2,color=b_purple,label=r'$d_{0,2}- \rho_s$')
    #plt.plot(times,np.ones(len(times))*vessels[0].ontology.B[0]/2,color=b_green,linestyle='--',label='Safe distance',linewidth=2)
    plt.grid('minor')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'$d_{i} - \rho_s$ [m]')
    #plt.legend()
    plt.savefig('output/distance_vv_snapshots.png')
    plt.figure()
    plt.plot(t[0], [psi*180/np.pi for psi in psi[0]],color=b_blue)
    plt.plot(t[1], [psi*180/np.pi for psi in psi[1]],color=b_green)
    plt.plot(t[2], [psi*180/np.pi for psi in psi[2]],color=b_purple)
    plt.grid('minor')
    plt.xlabel('Time [sec]')
    plt.ylabel(r'$\psi$ [deg]')
    plt.savefig('output/heading_angle_vv_snapshots.png')
    plt.show()
    # colors = ['blue', 'red', 'green']
    # cmap_cyan = ListedColormap(['none', b_cyan])
    
    # # --- Define the scenarios for each subplot ---
    # scenarios = [
    #     {
    #         'ax': axes[0],
    #         'time': snapshot_times[0],
    #         'title': f'Encounter 1: MyVessel vs. OtherVessel[0]\nat T = {snapshot_times[0]:.1f}s (Min TCPA)',
    #         'vessel_indices': [0, 1] # Indices for myvessel and othervessel[0]
    #     },
    #     {
    #         'ax': axes[1],
    #         'time': snapshot_times[1],
    #         'title': f'Encounter 2: MyVessel vs. OtherVessel[1]\nat T = {snapshot_times[1]:.1f}s (Min TCPA)',
    #         'vessel_indices': [0, 2] # Indices for myvessel and othervessel[1]
    #     }
    # ]

    # for scenario in scenarios:
    #     ax = scenario['ax']
    #     t_snap = scenario['time']
    #     img = plt.imread('assets/vessel.png')
    #     # Find the simulation index closest to the snapshot time
    #     time_idx = np.argmin(np.abs(np.array(T_vals[0]) - t_snap))

    #     # Plot for each vessel involved in the current scenario
    #     for v_idx in scenario['vessel_indices']:
    #         vessel = vessels[v_idx]
    #         color = colors[v_idx]

    #         # 1. Plot the base reference route
    #         #ax.plot(vessel.route[0], vessel.route[1], '--', color=color, alpha=0.4, label=f'_nolegend_')

    #         # 2. Plot the calculated path envelope at this snapshot
    #         current_dx = Dx_history[v_idx][time_idx]
    #         current_dy = Dy_history[v_idx][time_idx]
    #         envelope_x = vessel.route[0] + current_dx
    #         envelope_y = vessel.route[1] + current_dy
    #         if v_idx == 0:
    #             ax.plot(envelope_x, envelope_y, '-', color=b_blue, lw=2, label=f'Vessel {v_idx} Envelope')
    #         elif v_idx == 1:
    #             ax.plot(envelope_x, envelope_y, '-', color=b_red, lw=2, label=f'Vessel {v_idx} Envelope')
    #         else:
    #             ax.plot(envelope_x, envelope_y, '-', color=b_red, lw=2, label=f'Vessel {v_idx} Envelope')
    #         #ax.set_xlim(-10, 6010)
    #         #ax.set_ylim(1800, 2300)
    #         # 3. Plot the vessel's actual position (the "solution")
    #         rot_img = ndimage.rotate(img, psi_vals[v_idx][time_idx]*180/np.pi)
    #         #ax.plot(X_vals[v_idx][time_idx], -Y_vals[v_idx][time_idx], 'o', color=b_red, markersize=12,label=f'Vessel {v_idx} Position')
    #         ax.imshow(rot_img, aspect='auto',extent=(X_vals[v_idx][time_idx]-vessel.ontology.L[0]/2,X_vals[v_idx][time_idx]+vessel.ontology.L[0]/2,-Y_vals[v_idx][time_idx]-vessel.ontology.B[0]/2,-Y_vals[v_idx][time_idx]+vessel.ontology.B[0]/2))
    #         # 4. Plot the optimization constraint circles/ellipses
    #         # This visualizes the space the optimizer could choose from at each waypoint.
    #         # We assume the constraint is a circle with radius `look_ahead_distance`.
    #         # If your 'a' and 'b' constraints are different, you'd draw an ellipse.
    #         try:
    #             # Find the index of the first waypoint ahead of the vessel's current x-position
    #             psi=psi_vals[v_idx][time_idx]
    #             if np.cos(psi)[0] > 0:
    #                 ahead_indices = np.where(vessel.route[0]>= X_vals[v_idx][time_idx])[0]
    #             else:
    #                 ahead_indices = np.where(vessel.route[0]<= X_vals[v_idx][time_idx])[0]
    #             if len(ahead_indices) > 0:
    #                 waypoint_idx = ahead_indices[0]
    #             else:
    #                 # If no waypoints are ahead, the vessel is at or past the end
    #                 waypoint_idx = len(vessel.route[0]) - 1
    #         except IndexError:
    #             # If the vessel is past the last waypoint, use the last one
    #             waypoint_idx = len(vessel.route[0]) - 1
    #         for i in range(len(vessel.route[0])):
    #             vv = np.linspace(-look_ahead_distance,look_ahead_distance,1000)
    #             av,cv = np.meshgrid(vv,vv)
    #             center = (vessel.route[0][i], vessel.route[1][i])
    #             other_v_idx = [idx for idx in scenario['vessel_indices'] if idx != v_idx][0]
    #             xo=X_vals[other_v_idx][time_idx]
    #             yo=Y_vals[other_v_idx][time_idx]
    #             psi_o=psi_vals[other_v_idx][time_idx]
    #             rho_s=vessel.ontology.B[0]/2#myvessel.ontology.L[0]*dsafe
    #             rho_p= vessel.ontology.L[0]/2
    #             rho_q= vessel.ontology.L[0]/2
    #             if (i>=waypoint_idx):           
    #                 phi_1=(rho_s+rho_p+rho_q)*np.sqrt(1+np.tan(psi_o)**2)+ np.sign(-np.cos(vessel.y[-1]))*((yo-vessel.route[1][i])+(vessel.route[0][i]-xo)*np.tan(psi_o))
    #                 #plt.imshow(( (np.sign(-np.cos(vessel.y[-1]))*(cv-av*np.tan(psi_o))>= phi_1) & (np.sign(np.cos(vessel.y[-1]))*av>=0)).astype(int) ,extent=(av.min()+center[0],av.max()+center[0],cv.min()+center[1],cv.max()+center[1]),origin="lower", cmap="Greys", alpha = 0.3);
    #             # plt.show()
    #             # Center of the circle is the original waypoint
                
    #             # The radiu
    #             #radius = look_ahead_distance
                
    #             # Draw a circle representing the feasible set for (dx, dy)
    #             # We draw it around the *deviated* point for clarity
    #             #constraint_patch = Ellipse(xy=(vessel.route[0][i], vessel.route[1][i]), width=radius*2, height=radius*2,
    #             #                          edgecolor=b_cyan, facecolor='none', linestyle=':', alpha=0.4)
    #             #ax.add_patch(constraint_patch)

    #     #ax.set_title(scenario['title'], fontsize=14)
    #     ax.set_xlabel("x (m)")
    #     ax.set_ylabel("y (m)")
    #     ax.grid('minor')
    #     #ax.legend()
    #     #ax.axis('equal')

    # #plt.tight_layout()
    # plt.savefig("detailed_encounter_snapshots.png")
    plt.show()
    pos=next(x[0] for x in enumerate(dbank1) if x[1] > 10**(-3))
    pos=max(pos,next(x[0] for x in enumerate(dbank2) if x[1] > 10**(-3)))
    plt.figure()
    plt.plot(t[0][pos:], dbank1[pos:],color=b_blue,label='Port bank distance')
    plt.plot(t[0][pos:], dbank2[pos:],color='red',label='Starboard bank distance')
    plt.plot(t[0][pos:], dbanko1[pos:],color=b_purple,linestyle='-.',label='Port bank distance')
    #print(dbanko1[pos:])
    plt.plot(t[0][pos:], dbanko2[pos:],color='orange',linestyle='-.',label='Starboard bank distance')
    plt.plot(t[0][pos:],0.8*750*np.ones(len(t[0][pos:])), color=b_green,linestyle='--')
    plt.plot(t[0][pos:],0.2*750*np.ones(len(t[0][pos:])), color=b_green,linestyle='--')
    #plt.plot(t[0],-(np.array(dbank1)+np.array(dbank2))/2, 'g--', label='Safe distance')
    plt.grid('minor')
    plt.xlabel('Time [sec]')
    plt.ylabel('Own vessel bank distance[m]')
    #plt.legend()
    plt.savefig('output/bank_distance_vessel.png')
    plt.show()