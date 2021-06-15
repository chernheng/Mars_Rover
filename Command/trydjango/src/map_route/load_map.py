import numpy as np
import math

from map_route.occupancyGrid import OccupancyGridMap
from map_route.rover import Rover

from map_route.utils import file_read

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

# # load the map
def get_initial_map_array():
    ox, oy, obj_color, obj_coord_list = file_read("map_route/map_data.csv")
    
    map_size = 100
    xyreso = 5  # x-y grid resolution
    bias = math.floor(0.5*map_size*xyreso)
    # setting defualt grid value to 0.5 - unknown space
    map_array = np.ones((map_size, map_size)) * unknown_region
    gmap = OccupancyGridMap(map_array, xyreso, bias)

    rover_start_coord  = float(math.floor(0.5*map_size*xyreso))
    rover_obj = Rover(25,20,rover_start_coord, rover_start_coord,15,90)
    gmap.set_boundary(ox, oy, obj_color)
    gmap.flood_fill(rover_obj)
    return gmap.data, rover_obj, gmap, obj_coord_list, rover_start_coord
