import csv
import numpy as np
import pygame
from pyproj import Transformer
import geopandas as gpd
import copy
import os
import pygame
import csv
from shapely.geometry import box
import numpy as np
import xml.etree.ElementTree as ET
import math
from channel import get_channel_depth
import colorsys


def draw_ellipse_angle(surface, color, rect, angle, width=0):
    """
    Draws an ellipse on a given surface with a specified angle of rotation.
    Parameters:
        surface (pygame.Surface): The surface on which to draw the ellipse.
        color (tuple): The color of the ellipse, in RGB format.
        rect (tuple): A tuple representing the rectangle that bounds the ellipse (x, y, width, height).
        angle (float): The angle by which to rotate the ellipse, in degrees.
        width (int, optional): The width of the ellipse's border. Defaults to 0 for a filled ellipse.
    Returns:
        None
    """

    target_rect = pygame.Rect(rect)
    shape_surf = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.ellipse(shape_surf, color, (0, 0, *target_rect.size), width)
    rotated_surf = pygame.transform.rotate(shape_surf, angle)
    surface.blit(rotated_surf, rotated_surf.get_rect(center = target_rect.center))

def draw_vessels(screen, players, sim_scale, safe_distance_L, time, times_to_draw,minx, miny, maxx, maxy):
    WIDTH,HEIGHT=1200,800
    for ii in range(0,len(players)):
        instance=players[ii]
        time=times_to_draw[ii]
        MIN_LENGTH_PX = 16
        MIN_BEAM_PX = 8
        if (time<=instance.time):
            # print("I am at point [m] ",instance.state[3],instance.state[4])
            ship_x,ship_y =to_screen(copy.deepcopy(instance.state[3]),copy.deepcopy(instance.state[4]),minx, maxx, miny, maxy)# 1. Transform position to map
            # print("I am getting drawn at point",ship_x,ship_y, f", bounds are [{minx},{maxx}] and [{miny}, {maxy}]")
            #print(ship_x)
            draw_ellipse_angle(screen, "blue", [ship_x-np.max([sim_scale*instance.ontology.L[0],MIN_LENGTH_PX])/2, ship_y-np.max([sim_scale*instance.ontology.B[0],MIN_BEAM_PX])/2, np.max([sim_scale*instance.ontology.L[0],MIN_LENGTH_PX]), np.max([sim_scale*instance.ontology.B[0],MIN_BEAM_PX])], instance.ontology.psi[-1]*180/np.pi)
            # 3. Draw safe distance
            pygame.draw.circle(screen, "orange", (ship_x, ship_y), sim_scale*(safe_distance_L+np.max([instance.ontology.L[0],MIN_LENGTH_PX/sim_scale])/2), 1)
            # 4. Draw ship domain
            pygame.draw.circle(screen, "red", (ship_x, ship_y), np.max([sim_scale*instance.ontology.L[0],MIN_LENGTH_PX])/2,1)
            # 5. Draw encounter radius
            pygame.draw.circle(screen, "cyan", (ship_x, ship_y),np.max([sim_scale*5*instance.ontology.L[0],5*MIN_LENGTH_PX]), 1)
            # 6. Draw the path and active segment
            #draw_route(screen, sim_scale, instance)
            #pygame.display.flip()


def draw_vessels_no_map(screen, players, sim_scale,
                        safe_distance_L, time,
                        times_to_draw, look_ahead):

    n_players = max(1, len(players))

    for ii, instance in enumerate(players):

        time = times_to_draw[ii]

        if time <= instance.time:

            ship_x = sim_scale * instance.ontology.x[-1]
            ship_y = -sim_scale * instance.ontology.y[-1]

            # Generate a unique bright color
            hue = ii / n_players

            # Skip blue/cyan region (water background)
            if 0.45 <= hue <= 0.70:
                hue = (hue + 0.30) % 1.0

            r, g, b = colorsys.hsv_to_rgb(
                hue,
                0.95,   # saturation
                1.0     # brightness
            )

            vessel_color = (
                int(r * 255),
                int(g * 255),
                int(b * 255)
            )
            draw_route(screen, sim_scale, instance)
            draw_ellipse_angle(
                screen,
                vessel_color,
                [
                    ship_x - sim_scale*instance.ontology.L[0] / 2,
                    ship_y - sim_scale*instance.ontology.B[0] / 2,
                    sim_scale*instance.ontology.L[0],
                    sim_scale*instance.ontology.B[0]
                ],
                instance.ontology.psi[-1] * 180 / np.pi
            )

            # Optional: draw player number
            # font.render(str(ii+1), True, (0,0,0))


def draw_route(screen, sim_scale,  instance,*args):
    #for k in range(0,len(instance.route[0])-1):
    #print(args[0][0][0])
    if args and (args[0][0]==1):
        WIDTH,HEIGHT=1200,800
        transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)
        pixel_route = []
        for ii in range(len(instance.route[0])):
            lon = instance.route[0][ii]
            lat = instance.route[1][ii]
            
            # Convert GPS degrees to Mercator meters
            mx, my = transformer.transform(lon, lat)
            
            # --- THE VITAL FIX ---
            # If your map background looks normal but your routes are shifted down/inverted,
            # we force the Pygame Y-pixel mapping to scale from the TOP of the frame:
            sx = int((mx - args[0][1]) * (WIDTH / (args[0][3] - args[0][1])))
            
            # Try switching this Y calculation to see which matches your specific background layer:
            # Option A (Standard Geography inversion):
            sy = int(HEIGHT - ((my - args[0][2]) * (HEIGHT / (args[0][4] - args[0][2]))))
            
            # Option B (If your map background logic doesn't use standard inversion):
            # sy = int((my - map_config.miny) * (HEIGHT / (map_config.maxy - map_config.miny)))
            
            pixel_route.append((sx, sy))
            
        # Draw the paths
        line_color = "black" #if instance.name == "Other2" else "red"
        for ii in range(len(pixel_route) - 1):
            pygame.draw.line(screen, line_color, pixel_route[ii], pixel_route[ii+1], 3)
            pygame.draw.circle(screen, "black", pixel_route[ii], 8, 1)
            
        if pixel_route:
            pygame.draw.circle(screen, "black", pixel_route[-1], 8, 1)
        #print(f"Route point {ii} mapped to pixel: {screen_x}, {screen_y}")
    else:
        route_points=[np.array([instance.route[0][ii], instance.route[1][ii]]) for ii in range(0,len(instance.route[0]))]
        distance_from_current=[np.sqrt((route_points[jj][0] - instance.ontology.yx[-1])**2 + (route_points[jj][1] - instance.ontology.yy[-1])**2 ) for jj in range(0,len(route_points))]
        k=np.argmin(distance_from_current)
        for ii in range (0,len(instance.route[0])-1):
            pygame.draw.line(screen, "gray", (sim_scale*instance.droute[0][ii]+sim_scale*instance.route[0][ii] ,-sim_scale*instance.route[1][ii]-sim_scale*instance.droute[1][ii] ), (sim_scale*instance.droute[0][ii+1]+sim_scale*instance.route[0][ii+1] ,-sim_scale*instance.route[1][ii+1]-sim_scale*instance.droute[1][ii+1] ), 2)
            # for k in range(len(instance.route[0])):
            #     pygame.draw.circle(screen,"red",(sim_scale*instance.route[0][k],-sim_scale*instance.route[1][k]),2)
        if (k<len(instance.route[0])-1):
            #pygame.draw.line(screen, "red", (sim_scale*instance.droute[0][k]+sim_scale*instance.route[0][k] ,-sim_scale*instance.route[1][k]-sim_scale*instance.droute[1][k] ), (sim_scale*instance.droute[0][k+1]+sim_scale*instance.route[0][k+1] ,-sim_scale*instance.route[1][k+1]-sim_scale*instance.droute[1][k+1]), 2)
            #pygame.draw.circle(screen,'black',(sim_scale*instance.droute[0][k]+sim_scale*instance.route[0][k] ,-sim_scale*instance.route[1][k]-sim_scale*instance.droute[1][k] ),5,1)
            #pygame.draw.circle(screen,'black',(sim_scale*instance.droute[0][k+1]+sim_scale*instance.route[0][k+1] ,-sim_scale*instance.route[1][k+1]-sim_scale*instance.droute[1][k+1] ), 5,1)
            instance.arrived=0
        else:
            instance.arrived=1
        #pygame.display.flip()

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


def fetch_map(screen,MIN_LON, MAX_LON,MIN_LAT, MAX_LAT,water_names=[], map_type="global"):
    CACHE_DIR = "assets/"
    bounds_geom = box(MIN_LON, MIN_LAT, MAX_LON, MAX_LAT)
    if (map_type=='global'):
        land_local_path = os.path.join(CACHE_DIR, "ne_10m_land.geojson")
        lakes_local_path = os.path.join(CACHE_DIR, "ne_10m_lakes.geojson")
        
        land_url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_land.geojson"
        lakes_url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_lakes.geojson"
        if os.path.exists(land_local_path):
            print("Loading land data from local cache...")
            world = gpd.read_file(land_local_path)
        else:
            print("Land map data not found locally. Downloading from GitHub (this may take a moment)...")
            world = gpd.read_file(land_url)
            # Save it locally so the next run is near-instant
            world.to_file(land_local_path, driver="GeoJSON")
            print("Land data saved to cache.")

        # --- 2. LOAD OR DOWNLOAD LAKES DATA ---
        if os.path.exists(lakes_local_path):
            print("Loading lakes data from local cache...")
            lakes = gpd.read_file(lakes_local_path)
        else:
            print("Inland water map data not found locally. Downloading from GitHub...")
            lakes = gpd.read_file(lakes_url)
            # Save it locally
            lakes.to_file(lakes_local_path, driver="GeoJSON")
            print("Lakes data saved to cache.")
        geo_data = gpd.clip(world, bounds_geom)
        geo_data = geo_data.to_crs(epsg=3857)
        lakes=gpd.clip(lakes,bounds_geom)
        lakes = lakes.to_crs(epsg=3857)
        if (water_names!=[]):
            inland_water=lakes[lakes['name'].isin(water_names)]
        else:
            inland_water=[]
    elif (map_type=="nl_high_res"):
        nl_water_url = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_geography_marine_polys.geojson" 
        nl_water_local = os.path.join(CACHE_DIR, "nl_high_res_water.geojson")
        
        if os.path.exists(nl_water_local):
            print("Loading high-detail NL water from local cache...")
            nl_water = gpd.read_file(nl_water_local)
        else:
            print("Downloading high-detail water dataset...")
            # Note: For strict port engineering lines, a local country file is best. 
            # We fetch and cache it right here.
            nl_water = gpd.read_file(nl_water_url)
            nl_water.to_file(nl_water_local, driver="GeoJSON")
            
        # For high-res water layouts, we clip the water to our bounds
        nl_water = gpd.clip(nl_water, bounds_geom)
        
        if not nl_water.empty:
            nl_water = nl_water.to_crs(epsg=3857)
            inland_water = gpd.GeoDataFrame(geometry=[nl_water.unary_union], crs="epsg:3857")
        else:
            inland_water = gpd.GeoDataFrame(geometry=[], crs="epsg:3857")
        # Create a solid land background bounding frame
        geo_data = gpd.GeoDataFrame(geometry=[bounds_geom], crs="epsg:4326").to_crs(epsg=3857)
    if map_type == "global" and not geo_data.empty:
        minx, miny, maxx, maxy = geo_data.total_bounds
    else:
        # For zoomed local harbor frames, lock scaling strictly to the input window coordinates
        land_box_merc = gpd.GeoDataFrame(geometry=[bounds_geom], crs="epsg:4326").to_crs(epsg=3857)
        minx, miny, maxx, maxy = land_box_merc.total_bounds
    return geo_data,inland_water, minx, miny, maxx, maxy

def draw_map(screen,minx, miny, maxx, maxy,geo_data,lake_data):
    SEA_BLUE = (220, 240, 255)    # Very light blue sea
    LAND_GREEN = (245, 250, 240)  # Off-white/very pale green land
    BORDER_COLOR = (180, 180, 180) # Subtle grey borders
    #print("Creting simulation world...")
    screen.fill(SEA_BLUE)
    #print("Adding land...")
    buoys=[]
    for geom in geo_data.geometry:
        polys = list(geom.geoms) if geom.geom_type == 'MultiPolygon' else [geom]

        for poly in polys:
            # Draw Land
            exterior_points = [to_screen(x, y,minx, maxx, miny, maxy) for x, y in poly.exterior.coords]
            buoys.extend(exterior_points)
            if len(exterior_points) > 2:
                pygame.draw.polygon(screen, LAND_GREEN, exterior_points)
                pygame.draw.aalines(screen, BORDER_COLOR, True, exterior_points)
                
            # Draw Holes (Water inside land)
            for interior in poly.interiors:
                interior_points = [to_screen(x, y,minx, maxx, miny, maxy) for x, y in interior.coords]
                if len(interior_points) > 2:
                    pygame.draw.polygon(screen, SEA_BLUE, interior_points)
                    pygame.draw.aalines(screen, BORDER_COLOR, True, interior_points)
    #print("Adding inland water masses...")
    try:
        for geom in lake_data.geometry:
            polys = list(geom.geoms) if geom.geom_type == 'MultiPolygon' else [geom]
            for poly in polys:
                exterior_points = [to_screen(x, y,minx, maxx, miny, maxy) for x, y in poly.exterior.coords]
                buoys.extend(exterior_points)
                if len(exterior_points) > 2:
                    pygame.draw.polygon(screen, SEA_BLUE, exterior_points)
                for interior in poly.interiors:
                    interior_points = [to_screen(x, y,minx, maxx, miny, maxy) for x, y in interior.coords]
                    pygame.draw.polygon(screen, SEA_BLUE, interior_points)
    except:
        pass
        #print("No inland masses specified, so no inland masses were included in the map!")
    pygame.display.flip()
    return buoys

def to_world(px, py, minx, miny, maxx, maxy):
    HEIGHT, WIDTH = 800,1200
    # 1. Remember if the original input was a single scalar
    is_scalar = np.isscalar(px)
    
    # 2. Convert to numpy arrays so the math works smoothly for lists/arrays/scalars
    px_arr = np.asarray(px)
    py_arr = np.asarray(py)
    
    # 3. Perform the math using the numpy arrays
    y = miny + (HEIGHT - py_arr) * ((maxy - miny) / HEIGHT)
    x = minx + px_arr * ((maxx - minx) / WIDTH)
    
    # 4. Clean up the output type based on the original input
    if is_scalar:
        return float(x), float(y)
    else:
        return x.astype(float).tolist(), y.astype(float).tolist()

def create_inland_depth_varying_map(screen, instance, font, sim_scale, min_depth=3.1, max_depth=3.3):
    """
    Renders a uniform-width segment of the Maas River.
    Uses projected vertical bounding boxes to eliminate edge chopping.
    """
    SCREEN_WIDTH, SCREEN_HEIGHT = screen.get_size()
    
    # Use sim_scale to convert the 24m channel width into pixels consistently
    channel_width_meters = 40.8
    w_pixels = channel_width_meters * sim_scale
    half_w = w_pixels / 2
    
    COLOR_LAND_BEIGE = (225, 204, 166)
    COLOR_SHORELINE  = (100, 130, 160)
    
    screen.fill(COLOR_LAND_BEIGE)
    pixel_array = pygame.PixelArray(screen)
    
    shore_top = []
    shore_bottom = []
    
    model_min = 2.0
    model_max = 3.8
    depth_range = model_max - model_min

    # Pre-calculate arrays for the centerline
    x_vals = np.arange(SCREEN_WIDTH)
    norm_x = x_vals / SCREEN_WIDTH
    # Note: the centerline varies slowly around 0.5 * SCREEN_HEIGHT
    # with one smooth peak across the full window.

    y_c_vals = (
        0.5
        + 0.2 * np.sin(1*norm_x * np.pi)
    ) * SCREEN_HEIGHT

    dy_dnormx = (
        +0.2
        * np.pi
        * np.cos(1*norm_x * np.pi)
    )

    dy_dx = (
        dy_dnormx
        * (SCREEN_HEIGHT / SCREEN_WIDTH)
    )

    # Calculate expanded vertical thickness required
    # for constant perpendicular width
    cos_theta = (
        1.0
        / np.sqrt(
            1.0 + dy_dx**2
        )
    )

    vert_half_w = (
        half_w
        / cos_theta
    )
    # # Note: The centerline oscillates around 0.5 * SCREEN_HEIGHT. 
    # # This aligns perfectly with the (SCREEN_HEIGHT / 2) offset used in the vessel drawing!
    # y_c_vals = (0.5 + 0.12 * np.sin(norm_x * 2 * np.pi) + 0.04 * np.sin(norm_x * 5.5 * np.pi)) * SCREEN_HEIGHT
    
    # dy_dnormx = (0.12 * 2 * np.pi * np.cos(norm_x * 2 * np.pi) + 
    #              0.04 * 5.5 * np.pi * np.cos(norm_x * 5.5 * np.pi))
    # dy_dx = dy_dnormx * (SCREEN_HEIGHT / SCREEN_WIDTH)
    
    # # Calculate expanded vertical thickness required for constant perpendicular width
    # cos_theta = 1.0 / np.sqrt(1.0 + dy_dx**2)
    # vert_half_w = half_w / cos_theta

    for x in range(SCREEN_WIDTH):
        y_c = y_c_vals[x]
        v_hw = vert_half_w[x]
        
        # Perfect vertical bounds guaranteed for every X column (No chopping)
        y_start = int(y_c - v_hw)
        y_end = int(y_c + v_hw)
        
        shore_top.append((x, y_start))
        shore_bottom.append((x, y_end))
        
        y_start_draw = max(0, y_start)
        y_end_draw = min(SCREEN_HEIGHT, y_end)
        
        for y in range(y_start_draw, y_end_draw):
            # The API function now internally accounts for the curve geometry
            # Ensure get_channel_depth is defined elsewhere in your code
            depth = get_channel_depth(x, y, SCREEN_WIDTH, SCREEN_HEIGHT, min_depth, max_depth)
            
            if depth_range > 0:
                color_factor = (depth - model_min) / depth_range
            else:
                color_factor = 0.5
            color_factor = max(0.0, min(1.0, color_factor))
            
            r = int(190 - (190 - 10) * color_factor)
            g = int(230 - (230 - 50) * color_factor)
            b = int(255 - (255 - 130) * color_factor)
            
            pixel_array[x, y] = (r, g, b)
            
    del pixel_array
    
    if len(shore_top) > 1:
        pygame.draw.aalines(screen, COLOR_SHORELINE, False, shore_top)
    if len(shore_bottom) > 1:
        pygame.draw.aalines(screen, COLOR_SHORELINE, False, shore_bottom)
        
    screen.blit(font.render('Time: '+str(round(instance.ontology.time[-1],2)), True, "white"), (0, 0))
    pygame.display.flip()
    
    return shore_top, shore_bottom

def to_screen(x, y,minx,maxx,miny,maxy):
    HEIGHT, WIDTH = 800, 1200
    px = (x - minx) * (WIDTH / (maxx - minx))
    py = HEIGHT - ((y - miny) * (HEIGHT / (maxy - miny)))
    return px, py

def parse_kml_to_pygame(kml_path, screen_width, screen_height):
    tree = ET.parse(kml_path)
    root = tree.getroot()
    
    # Define the KML namespace to locate tags correctly
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    
    # Locate the coordinates string inside the Polygon tag
    coords_text = root.find('.//kml:coordinates', ns).text.strip()
    
    # Parse the string into a list of (longitude, latitude) tuples
    raw_points = []
    for pt in coords_text.split():
        lon, lat, _ = map(float, pt.split(','))
        raw_points.append((lon, lat))
        
    # --- Scaling to Pygame Screen ---
    lons = [p[0] for p in raw_points]
    lats = [p[1] for p in raw_points]
    min_lon, max_lon = min(lons), max(lons)
    min_lat, max_lat = min(lats), max(lats)
    
    # 1. Correct for the Earth's curvature (Equirectangular approximation)
    mean_lat_rad = math.radians((min_lat + max_lat) / 2.0)
    lon_correction = math.cos(mean_lat_rad)
    
    # Calculate the "corrected" range
    lon_range = (max_lon - min_lon) * lon_correction
    lat_range = max_lat - min_lat
    
    # 2. Use a single uniform scale factor to prevent stretching
    scale = min(screen_width / lon_range, screen_height / lat_range)
    
    # 3. Calculate X and Y offsets to keep it centered on screen
    x_offset = (screen_width - (lon_range * scale)) / 2
    y_offset = (screen_height - (lat_range * scale)) / 2  # Added Y offset
    
    # Convert geographic points to pixel points
    screen_points = []
    for lon, lat in raw_points:
        # Apply the longitude correction, scale, and offset for X
        screen_x = x_offset + ((lon - min_lon) * lon_correction * scale)
        
        # Apply the scale and the new Y offset
        screen_y = y_offset + ((max_lat - lat) * scale)
        
        screen_points.append((screen_x, screen_y))
        
    # NEW: Return the offsets alongside the points
    return screen_points, x_offset, y_offset


def create_map(screen, font, map_csv, instance, polygon_points=None):
    x_data = []
    y_data = []
    
    # 1. Base Layer: Water Background
    screen.fill((207, 185, 151))  
    
    # 2. Secondary Layer: KML Operating Area
    if polygon_points:
        # Draw the port polygon in a slightly different shade (e.g., deeper blue or green)
        pygame.draw.polygon(screen, (135, 206, 250), polygon_points)
        # Optional: Draw the outline
        pygame.draw.polygon(screen, (70, 130, 180), polygon_points, width=2)

    # 3. Read Obstacle/Boundary Data
    with open(map_csv, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.reader(infile)
        next(reader)
        for row in reader:
            x_str = row[1].strip().strip("[]")
            y_str = row[2].strip().strip("[]")
            x_list = [float(x.strip()) for x in x_str.split(";")]
            y_list = [float(y.strip()) for y in y_str.split(";")]
            x_data.append(x_list)
            y_data.append(y_list)   
            
    # 4. Top Layer: Draw CSV Rectangles
    for ii in range(0, len(x_data)):
        if len(x_data[ii]) == 2:
            w = x_data[ii][1] - x_data[ii][0]
            h = y_data[ii][1] - y_data[ii][0]
            rect = pygame.Rect(x_data[ii][0], y_data[ii][0], w, h)
            pygame.draw.rect(screen, "black", rect)
            
    # 5. Telemetry Text Overlays
    screen.blit(font.render('Time: '+str(round(instance.ontology.time[-1],2)), True, "white"), (0, 0))
    # screen.blit(font.render('Rudder Command: '+str(round(instance.u[-1],2)), True, "white"), (0, 600))
    # screen.blit(font.render('Error: '+str(round(0,2)), True, "white"), (0,700))
    # screen.blit(font.render('Reference: '+str(round(instance.ontology.psi_d[-1],2)), True, "white"), (0,650))
    
    return x_data, y_data
