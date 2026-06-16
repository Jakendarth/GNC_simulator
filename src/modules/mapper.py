#import geopandas as gpd
#import networkx as nx
#import os
import csv
#from scipy.spatial import KDTree
#import matplotlib.pyplot as plt
import numpy as np
import ast
import pygame
#import matplotlib.animation as animation


def create_map(screen,font,map,instance):
    x_data=[]
    y_data=[]
    screen.fill((173, 216, 230))  # Light blue water background
    with open(map, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.reader(infile)
        next(reader)
        for row in reader:
            x_str = row[1].strip().strip("[]")
            y_str = row[2].strip().strip("[]")
            x_list = [float(x.strip()) for x in x_str.split(";")]
            y_list = [float(y.strip()) for y in y_str.split(";")]
            x_data.append(x_list)
            y_data.append(y_list)
    #print(x_data)
    #print(y_data)
    for ii in range(0,len(x_data)):
        if len(x_data[ii])==2:
            # Shape is Rectangle
            w=x_data[ii][1]-x_data[ii][0]
            h=y_data[ii][1]-y_data[ii][0]
            rect=pygame.Rect(x_data[ii][0], y_data[ii][0], w, h)
            pygame.draw.rect(screen, "black", rect)
    screen.blit(font.render('Time: '+str(round(instance.ontology.time[-1],2)), True, "white"), (0, 0))
    screen.blit(font.render('Rudder Command: '+str(round(instance.u[-1],2)), True, "white"), (0, 600))
    screen.blit(font.render('Time: '+str(round(instance.ontology.time[-1],2)), True, "white"), (0, 0))
    screen.blit(font.render('Error: '+str(round(instance.e[-1],2)), True, "white"), (0,700))
    screen.blit(font.render('Reference: '+str(round(instance.ontology.psi_d[-1],2)), True, "white"), (0,650))
    return x_data, y_data


def map_synthesis(start_point,end_point,shapefile):
    G=nx.Graph()
    gdf=gpd.read_file(shapefile)
    fig, ax = plt.subplots(figsize=(10, 10))
    for i, segment in enumerate(gdf.geometry):
            x, y = segment.xy
            points=[[x,y] for x, y in zip(x.tolist(), y.tolist())]
            ax.plot(x,y,'b-')
    '''
    searchfile=os.path.basename(shapefile)
    searchfile="assets/traffic_networks/"+searchfile.split('.')[0]+'.graphml'
    if not os.path.isfile(searchfile): 
        process_map(shapefile)
    G=nx.read_graphml(searchfile)
    G=G.to_undirected()
    pos = nx.spring_layout(G) 
    # Extract unique node coordinates for KDTree
    nodes = [tuple(map(float, node.strip('()').split(','))) for node in G.nodes]
    kd_tree = KDTree(nodes)  # For fast nearest-neighbor search

    # User-provided start and end points
    start_point = tuple(start_point)
    end_point = tuple(end_point)
    print("Start point:", start_point)
    print("End point:", end_point)

    # Find the nearest nodes in the network
    _, start_idx = kd_tree.query(start_point)
    _, end_idx = kd_tree.query(end_point)

    start_node = nodes[start_idx]
    end_node = nodes[end_idx]
    # Compute the shortest path using Dijkstra's algorithm
    start_node = str(start_node)
    end_node = str(end_node)
    shortest_path = nx.shortest_path(G, source=start_node, target=end_node, weight="weight")
    #print("Shortest path:", shortest_path)
    # Create the figure
    fig, ax = plt.subplots(figsize=(6, 6))

    # Draw the initial graph
    nx.draw(G, pos, with_labels=True, node_color="lightgray", edge_color="gray", ax=ax)

    # Animate shortest path discovery
    def update(frame):
        ax.clear()
        
        # Draw full graph
        nx.draw(G, pos, with_labels=True, node_color="lightgray", edge_color="gray", ax=ax)

        # Draw visited nodes up to current step
        nx.draw_networkx_nodes(G, pos, nodelist=shortest_path[:frame+1], node_color="blue", ax=ax)

        # Draw visited edges up to current step
        edges = list(zip(shortest_path[:frame], shortest_path[1:frame+1]))  # Pair consecutive nodes
        nx.draw_networkx_edges(G, pos, edgelist=edges, edge_color="red", width=2, ax=ax)

        # Highlight start and end nodes
        nx.draw_networkx_nodes(G, pos, nodelist=[start_point], node_color="green", node_size=500, ax=ax)
        nx.draw_networkx_nodes(G, pos, nodelist=[end_point], node_color="orange", node_size=500, ax=ax)

        # Show edge weights
        labels = nx.get_edge_attributes(G, "weight")
        nx.draw_networkx_edge_labels(G, pos, edge_labels=labels, ax=ax)

        ax.set_title(f"Step {frame+1}/{len(shortest_path)}")

    # Run animation
    ani = animation.FuncAnimation(fig, update, frames=len(shortest_path), interval=1000, repeat=False)
    '''
    plt.show()
    # Convert node strings back to tuples for plotting
    #shortest_path_coords = [tuple(map(float, node.strip("()").split(", "))) for node in shortest_path]
    #x_values, y_values = zip(*shortest_path_coords)
    #plt.plot(x_values,y_values, "ro-", label="Shortest Path")  # Red path
    #plt.plot(start_point[0], start_point[1], "mx", label="Start Point")  # Blue start
    #plt.plot(end_point[0], end_point[1], "gx", label="End Point")  # Green end
    #nx.draw(G, cmap = plt.get_cmap('jet'))
    #plt.show()

# def process_map(shapefile):
#     G=nx.Graph()
#     gdf=gpd.read_file(shapefile)
#     for i, segment in enumerate(gdf.geometry):
#         x, y = segment.xy
#         points=[[x,y] for x, y in zip(x.tolist(), y.tolist())]
#         print(gdf['vrt_naam'][i])
#         for j in range(0,len(points)-1):
#             for k in range(0,len(points)-1):
#                 if j!=k:
#                     G.add_edge(tuple(points[j]),tuple(points[k]),weight=np.sqrt((points[j][0]-points[k][0])**2+(points[j][1]-points[k][1])**2),name=gdf['vrt_naam'][i])
#     print(G.number_of_nodes())
#     print(G.number_of_edges())
#     filename=os.path.basename(shapefile)
#     filename="assets/traffic_networks/"+filename.split('.')[0]+".graphml"
#     nx.write_graphml(G, filename)