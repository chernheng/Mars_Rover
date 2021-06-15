import numpy as np
import math
import matplotlib.pyplot as plt


# grid map values
unknown_region = 1
known_empty_region = 0
known_boundary = 1.9
extrapolated_boundary = 1.5
rover_region = 1.9
# object colours
obj_value_grid = {}
obj_value_grid['r']=8
obj_value_grid['b']=6
obj_value_grid['g']=4
obj_value_grid['y']=3.1

def file_read(f):
    map_size = 100
    xyreso = 5  # x-y grid resolution
    bias = math.floor(0.5*map_size*xyreso)
    
    # csv file format with 2 columns - Col 1: angle (degrees); Col 2: distance (magn); Col 3: object color
    measures = [line.split(",") for line in open(f)]
    angles = []
    distances = []
    obj_color = []
    obj_coord_list = []
    for measure in measures:
        if (float(measure[1])+15)>=250:
            continue
        # convert from bearings to angle
        deg = 90.0 - float(measure[0])
        if deg<0:
            deg = deg + 360
        elif deg>=360:
            deg = deg - 360

        angles.append(deg)
        distances.append(float(measure[1])+15)
        obj_color.append(int(measure[2]))
        if int(measure[2])>=1 and int(measure[2])<=5:     
            obj_ox = np.cos(math.radians(deg)) * (float(measure[1])+15)
            obj_oy = np.sin(math.radians(deg)) * (float(measure[1])+15)
            obj_coord_list.append([obj_ox, obj_oy, int(measure[2])])
            
    angles = np.array(angles)
    distances = np.array(distances)
    obj_color = np.array(obj_color)

    ox = np.cos(np.radians(angles)) * distances
    oy = np.sin(np.radians(angles)) * distances
    return ox, oy, obj_color, obj_coord_list

#  /**
#      * @return true iff there is line-of-sight from (x1,y1) to (x2,y2).
#      */
def line_of_sight(gmap, x1, y1, x2, y2):
    dy = y2 - y1
    dx = x2 - x1

    f = 0

    signY = 1
    signX = 1
    offsetX = 0
    offsetY = 0
    
    if (dy < 0):
        dy *= -1
        signY = -1
        offsetY = -1
    if (dx < 0):
        dx *= -1
        signX = -1
        offsetX = -1
    
    if (dx >= dy):
        while (x1 != x2):
            f += dy
            if (f >= dx):
                if (gmap.is_occupied((x1 + offsetX, y1 + offsetY))):
                    return False
                y1 += signY
                f -= dx
            if (f != 0 and gmap.is_occupied((x1 + offsetX, y1 + offsetY))):
                return False
            if (dy == 0 and gmap.is_occupied((x1 + offsetX, y1)) and gmap.is_occupied((x1 + offsetX, y1 - 1))):
                return False
            
            x1 += signX
    else:
        while (y1 != y2):
            f += dx
            if (f >= dy):
                if (gmap.is_occupied((x1 + offsetX, y1 + offsetY))):
                    return False
                x1 += signX
                f -= dy
            if (f != 0 and gmap.is_occupied((x1 + offsetX, y1 + offsetY))):
                return False
            if (dx == 0 and gmap.is_occupied((x1, y1 + offsetY)) and gmap.is_occupied((x1 - 1, y1 + offsetY))):
                return False
            
            y1 += signY
        
    return True

def path_smoothing(gmap, vertices_list): 
    # returns list of path nodes which are straight line paths from one node to the other, disregarding the grids, while avoiding obstacles
    k = 0
    path_nodes = []
    path_nodes.append(vertices_list[0])
    for i in range(1,len(vertices_list)):
        if line_of_sight(gmap, path_nodes[k], vertices_list[i+1]) == False:
            k = k+1
            path_nodes[k] = vertices_list[i]
    k = k + 1
    path_nodes[k] = vertices_list[len(vertices_list)-1]
    return path_nodes

# function for line generation
# full four quadrants bresenham algorithm
def bresenham(x1,y1,x2,y2):
    if (x1<x2):
        x_dir = 1
        loop_end_x = x2+1
    else: 
        x_dir=-1
        loop_end_x = x2-1

    if (y1<y2):
        y_dir = 1
        loop_end_y = y2+1
    else: 
        y_dir=-1
        loop_end_y = y2-1

    dx = abs(x2-x1)
    dy = abs(y2-y1)
    line_path=[]

    if(dx>dy):
        m_new = 2 * dy
        slope_error_new = -dx

        y=y1
        for x in range(x1,loop_end_x,x_dir):
            line_path.append([x,y])
 
            # Add slope to increment angle formed
            slope_error_new =slope_error_new + m_new
 
            # Slope error reached limit, time to
            # increment y and update slope error.
            if (slope_error_new >= 0):
                y=y+y_dir
                slope_error_new =slope_error_new - 2 * dx

    else:
        m_new = 2 * dx
        slope_error_new = -dy
        
        x=x1
        for y in range(y1,loop_end_y,y_dir):
            line_path.append([x,y])
 
            # Add slope to increment angle formed
            slope_error_new =slope_error_new + m_new
 
            # Slope error reached limit, time to
            # increment y and update slope error.
            if (slope_error_new >= 0):
                x=x+x_dir
                slope_error_new =slope_error_new - 2 * dy 

    return line_path        

def plot_path(gmap, path):
    start_x, start_y = path[0]
    goal_x, goal_y = path[-1]

    # plot path
    path_arr = np.array(path)
    for i in path_arr:
        gmap.set_data_idx((i[0], i[1]), rover_region)

def get_index_from_coord(gmap, x, y):
    """
    Get the array indices of the given point.
    :param x: the point's x-coordinate in meters
    :param y: the point's y-coordinate in meters
    :return: the corresponding array indices as a (x, y) tuple
    """
    x_index = int(round(round(x/gmap.cell_size,3)))
    y_index = int(round(round(y/gmap.cell_size,3)))

    return x_index, y_index

def get_abs_angle(x1,y1, x2,y2):
    theta = math.atan(abs((y2-y1)/(x2-x1)))
    
    abs_ang = -1
    if x2>=x1:
        if y2>y1:
            abs_ang = 360 - math.degrees(theta)
        elif y2<=y1:
            abs_ang = math.degrees(theta)
    else:
        if y2>y1:
            abs_ang = 180 + math.degrees(theta)
        elif y2<=y1:
            abs_ang = 180 - math.degrees(theta)
    return abs_ang

def get_rover_command(gmap, path_px, rover, goal_node, rover_end_pos):
    initial_x, initial_y = rover.x_pos, rover.y_pos

    # [relative_angle, distance, last_index]
    instructions_list = []
    distance = 0
    path_arr = np.array(path_px)

    prev_x, prev_y = get_index_from_coord(gmap, rover.x_pos, rover.y_pos)
    print("Getting rover command...")
    print("start rover angle:", rover.angle)
    print("Starting index:", prev_x, prev_y)
    distance = 0
    prev_angle = rover.angle
    current_angle = rover.angle
    last_instr_track = 0
    for i in range(len(path_arr)-1):
        # current_angle = -1
        current_x, current_y = path_arr[i+1]
        print("Next index of rover:", current_x, current_y)
        if (current_x, current_y) != (prev_x, prev_y):
            if current_x > prev_x and current_y > prev_y:
                    current_angle = 315
            elif current_x > prev_x and current_y < prev_y:
                    current_angle = 45
            elif current_x < prev_x and current_y > prev_y:
                current_angle = 225
            elif current_x < prev_x and current_y < prev_y:
                    current_angle = 135
            elif current_x == prev_x and current_y > prev_y:
                current_angle = 270
            elif current_x == prev_x and current_y < prev_y:
                    current_angle = 90
            elif current_y == prev_y and current_x > prev_x:
                current_angle = 0
            elif current_y == prev_y and current_x < prev_x:
                current_angle = 180
            if current_angle == -1:
                print("New angle of rover not calculated")
            if current_angle != prev_angle:
                print("Abs angle of rover is changed to", current_angle)

                if i!=0:
                    instructions_list.append([0, distance*gmap.cell_size, i])
                distance = 0 # reset distance accumulation
                relative_ang = prev_angle-current_angle
                relative_ang_mod = relative_ang
                if relative_ang<-180:
                    relative_ang_mod = relative_ang+360
                elif relative_ang>180:
                    relative_ang_mod = relative_ang-360
        
                instructions_list.append([relative_ang_mod, 0, i])

                # this distance is in terms of cells
                if prev_angle==0 or prev_angle==90 or prev_angle==270 or prev_angle==360:
                    distance = distance + 1
                else:
                    distance = distance + math.sqrt(2)
                last_instr_track = 1
            else:
                print("Rover angle remained the same at", current_angle)
                # if angle is not changed - calculate the distance needed to travel
                # this distance is in terms of cells
                if current_angle==0 or current_angle==90 or current_angle==270 or current_angle==360:
                    distance = distance + 1
                else:
                    distance = distance + math.sqrt(2)
                last_instr_track = 2
        prev_angle = current_angle
        prev_x = current_x
        prev_y = current_y

    if distance != 0:
        instructions_list.append([0, distance*gmap.cell_size, len(path_arr)-1])

    # make final angle adjustment to face the object
    final_x_pos = rover_end_pos[0]
    final_y_pos = rover_end_pos[1]
    abs_ang = get_abs_angle(final_x_pos, final_y_pos, goal_node[0], goal_node[1])
    if abs_ang != -1:
        # no error in calculating absolute angle
        # when rover is not facing the object
        if current_angle != abs_ang:
            ang_calibration = current_angle-abs_ang
            ang_calibration_mod = ang_calibration
            if ang_calibration<-180:
                ang_calibration_mod = ang_calibration + 360
            elif ang_calibration>180:
                ang_calibration_mod = ang_calibration - 360

            instructions_list.append([ang_calibration_mod, 0, len(path_arr)-1])

    return instructions_list



