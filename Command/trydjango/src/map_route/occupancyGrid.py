import numpy as np
import matplotlib.pyplot as plt
import math
# from utils import png_to_ogm
from matplotlib import cm
from matplotlib.colors import ListedColormap
from collections import deque

from map_route.utils import bresenham

# grid map values
unknown_region = 1
known_empty_region = 0
known_boundary = 1.9
extrapolated_boundary = 1.5
rover_region = 1.9
# object colours corresponding values on the colorbar
obj_value_grid = [0,8,4,6,3.1,6.7]

class OccupancyGridMap:
    def __init__(self, data_array, cell_size, bias, occupancy_threshold=1.0):
        # Creates a grid map
        # data_array: a 2D array with an occupancy value for each cell (values from 0 - 1.9)
        # cell_size: cell size in cm (xyreso)
        # occupancy_threshold: A threshold to determine whether a cell is occupied or free.
        # A cell is considered occupied if its value >= occupancy_threshold, free otherwise.

        self.data = data_array
        self.dim_cells = data_array.shape
        self.dim_meters = (self.dim_cells[0] * cell_size, self.dim_cells[1] * cell_size)
        self.cell_size = cell_size
        self.occupancy_threshold = occupancy_threshold
        # 2D array to mark visited nodes (in the beginning, no node has been visited)
        self.visited = np.zeros(self.dim_cells, dtype=np.float32)
        self.axs = 0
        self.bias = bias
        self.fig = 0 

    def mark_visited_idx(self, point_idx):
        # Mark a point as visited.
        # point_idx: a point (x, y) in data array

        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.visited[y_index][x_index] = 1.0

    def mark_visited(self, point):
        # Mark a point as visited.
        # point: a 2D point (x, y) in meters
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.mark_visited_idx((x_index, y_index))

    def is_visited_idx(self, point_idx):
        # Check whether the given point is visited.
        # point_idx: a point (x, y) in data array
        # returns True if the given point is visited, false otherwise
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        if self.visited[y_index][x_index] == 1.0:
            return True
        else:
            return False

    def is_visited(self, point):
        # Check whether the given point is visited.
        # point: a 2D point (x, y) in meters
        # returns True if the given point is visited, false otherwise
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_visited_idx((x_index, y_index))

    def get_data_idx(self, point_idx):
        # Get the occupancy value of the given point.
        # point_idx: a point (x, y) in data array
        # returns the occupancy value of the given point
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        return self.data[y_index][x_index]

    def get_data(self, point):
        # Get the occupancy value of the given point.
        # point: a 2D point (x, y) in meters
        # returns: the occupancy value of the given point
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.get_data_idx((x_index, y_index))

    def set_data_idx(self, point_idx, new_value):
        # Set the occupancy value of the given point.
        # point_idx: a point (x, y) in data array
        # new_value: the new occupancy values
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            print("x index:", x_index)
            print("y index:", y_index)
            raise Exception('Point is outside map boundary')
            
        self.data[y_index][x_index] = new_value

    def set_data(self, point, new_value):
        # Set the occupancy value of the given point.
        # point: a 2D point (x, y) in meters
        # new_value: the new occupancy value
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        self.set_data_idx((x_index, y_index), new_value)

    def is_inside_idx(self, point_idx):
        """
        Check whether the given point is inside the map.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is inside the map, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            return False
        else:
            return True

    def is_inside(self, point):
        """
        Check whether the given point is inside the map.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is inside the map, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_inside_idx((x_index, y_index))

    def is_occupied_idx(self, point_idx):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is occupied, false otherwise
        """
        x_index, y_index = point_idx
        if self.get_data_idx((x_index, y_index)) >= self.occupancy_threshold:
            return True
        else:
            return False

    def is_occupied(self, point):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is occupied, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_occupied_idx((x_index, y_index))

    def is_unknown_idx(self, point_idx):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is occupied, false otherwise
        """
        x_index, y_index = point_idx
        if self.get_data_idx((x_index, y_index)) == unknown_region:
            return True
        else:
            return False
        
    def rover_collision_idx(self, rover):
        """
        checks whether rover area + border area is occupied by obstacles
        """
        # do a rough check first to reduce time of calculation

        # first checks whether borders of rovers collide with obstacles
        collision, index_x_list, index_y_list = self.check_rover_borders_on_grid(rover)
        # print("collision:", collision)
        if collision == True:
            return True
        # # if borders of rovers does not collide with obstacles - move on to check whether the area of the rover collide with obstacles
        fill_collision = self.check_rover_fill_collision(index_x_list, index_y_list)
        if fill_collision == True:
            return True
        return False
            
    def check_rover_borders_on_grid(self, rover):
        """
        checks whether rover border area is occupied by obstacles
        """
        # factor in extra space for uncertainties
        rover_width_widened = rover.width + 20
        rover_length_widened = rover.length + 20
        pivot_length_widened = rover.pivot_length + 10
        dist_centre_topcorners = math.sqrt((rover_length_widened-pivot_length_widened)*(rover_length_widened-pivot_length_widened)+(rover_width_widened/2)*(rover_width_widened/2))

        dist_centre_bottomcorners = math.sqrt((pivot_length_widened)*(pivot_length_widened)+(rover_width_widened/2)*(rover_width_widened/2))

        ang_topcorner_bottomcorner = math.degrees(math.atan((rover_width_widened/2) / (pivot_length_widened)))

        ang_front_topleft = math.degrees(math.atan((rover_width_widened/2) / (rover_length_widened-pivot_length_widened)))

        ang_back_to_bottomleft = math.degrees(math.atan((rover_width_widened/2) / (pivot_length_widened)))

        ang_topleft_bottomleft = 180 - ang_front_topleft - ang_back_to_bottomleft

        top_left_ang = ang_front_topleft  + rover.angle
        top_right_ang = rover.angle - ang_front_topleft

        bottom_left_ang = top_left_ang + ang_topleft_bottomleft
        bottom_right_ang = bottom_left_ang + ang_back_to_bottomleft*2

        top_left_dist = dist_centre_topcorners
        bottom_right_dist = dist_centre_bottomcorners

        top_left_x = np.cos(math.radians(top_left_ang)) * top_left_dist
        top_left_y = np.sin(math.radians(top_left_ang)) * top_left_dist
        top_right_x = np.cos(math.radians(top_right_ang)) * top_left_dist
        top_right_y = np.sin(math.radians(top_right_ang)) * top_left_dist

        bottom_left_x = np.cos(math.radians(bottom_left_ang)) * bottom_right_dist
        bottom_left_y = np.sin(math.radians(bottom_left_ang)) * bottom_right_dist
        bottom_right_x = np.cos(math.radians(bottom_right_ang)) * bottom_right_dist
        bottom_right_y = np.sin(math.radians(bottom_right_ang)) * bottom_right_dist
        
        top_left_index = self.get_index_from_coordinates(top_left_x+rover.x_pos,-top_left_y+rover.y_pos)
        top_right_index = self.get_index_from_coordinates(top_right_x+rover.x_pos,-top_right_y+rover.y_pos)
        bottom_left_index = self.get_index_from_coordinates(bottom_left_x+rover.x_pos,-bottom_left_y+rover.y_pos)
        bottom_right_index = self.get_index_from_coordinates(bottom_right_x+rover.x_pos,-bottom_right_y+rover.y_pos)

        rover_border_left = bresenham(top_left_index[0],top_left_index[1],bottom_left_index[0],bottom_left_index[1])
        rover_border_top = bresenham(top_left_index[0],top_left_index[1],top_right_index[0],top_right_index[1])
        rover_border_bottom = bresenham(bottom_left_index[0],bottom_left_index[1],bottom_right_index[0],bottom_right_index[1])
        rover_border_right = bresenham(top_right_index[0],top_right_index[1],bottom_right_index[0],bottom_right_index[1])

        index_x_list = []
        index_y_list = []

        collision = False

        for l in rover_border_left:
            if self.is_occupied_idx((l[0],l[1])) == False:
                index_x_list.append(l[0])
                index_y_list.append(l[1])
            else:
                # print("collision index:",l[0],l[1])
                collision = True
                # when collision is true, just exits function - index_x and index_y list are not needed to be used
                return collision, index_x_list, index_y_list
        for l in rover_border_top:
            if self.is_occupied_idx((l[0],l[1])) == False:
                index_x_list.append(l[0])
                index_y_list.append(l[1])
            else:
                collision = True
                # when collision is true, just exits function - index_x and index_y list are not needed to be used
                return collision, index_x_list, index_y_list
        for l in rover_border_bottom:
            if self.is_occupied_idx((l[0],l[1])) == False:
                index_x_list.append(l[0])
                index_y_list.append(l[1])
            else:
                collision = True
                # when collision is true, just exits function - index_x and index_y list are not needed to be used
                return collision, index_x_list, index_y_list
        for l in rover_border_right:
            if self.is_occupied_idx((l[0],l[1])) == False:
                index_x_list.append(l[0])
                index_y_list.append(l[1])
            else:
                collision = True
                # when collision is true, just exits function - index_x and index_y list are not needed to be used
                return collision, index_x_list, index_y_list
        return collision, index_x_list, index_y_list


    def check_rover_fill_collision(self, index_x_list, index_y_list):
        """
        checks whether rover area is occupied by obstacles
        """
        
        zipped_xy_list =  zip(index_x_list, index_y_list)
        sorted_index_xy_list = sorted(zipped_xy_list)
        sorted_index_line = {}
        collision = False
        for i in sorted_index_xy_list:
            if str(i[0]) in sorted_index_line:
                sorted_index_line[str(i[0])].append(i[1])
            else:
                list_y = [i[1]]
                sorted_index_line[str(i[0])] = list_y
        for i in sorted_index_line:
            list_y = sorted(sorted_index_line[i])
            sorted_index_line[i] = [list_y[0],list_y[-1]] # [min_y, max_y] for part. x coord
            # excluding the two extreme points where rover border is
            for j in range(list_y[-1]-list_y[0]-1):
                collision = self.is_occupied_idx((int(i), sorted_index_line[i][0]+j+1))
                if collision ==True:
                    return True
        return False
    
    def set_rover_borders_on_grid(self, rover):
        top_left_ang = rover.ang_front_topleft  + rover.angle
        top_right_ang = rover.angle - rover.ang_front_topleft

        bottom_left_ang = top_left_ang + rover.ang_topleft_bottomleft
        bottom_right_ang = bottom_left_ang + rover.ang_back_to_bottomleft*2

        top_left_dist = rover.dist_centre_topcorners
        bottom_right_dist = rover.dist_centre_bottomcorners

        top_left_x = np.cos(math.radians(top_left_ang)) * top_left_dist
        top_left_y = np.sin(math.radians(top_left_ang)) * top_left_dist
        top_right_x = np.cos(math.radians(top_right_ang)) * top_left_dist
        top_right_y = np.sin(math.radians(top_right_ang)) * top_left_dist

        bottom_left_x = np.cos(math.radians(bottom_left_ang)) * bottom_right_dist
        bottom_left_y = np.sin(math.radians(bottom_left_ang)) * bottom_right_dist
        bottom_right_x = np.cos(math.radians(bottom_right_ang)) * bottom_right_dist
        bottom_right_y = np.sin(math.radians(bottom_right_ang)) * bottom_right_dist
        
        top_left_index = self.get_index_from_coordinates(top_left_x+rover.x_pos,-top_left_y+rover.y_pos)
        top_right_index = self.get_index_from_coordinates(top_right_x+rover.x_pos,-top_right_y+rover.y_pos)
        bottom_left_index = self.get_index_from_coordinates(bottom_left_x+rover.x_pos,-bottom_left_y+rover.y_pos)
        bottom_right_index = self.get_index_from_coordinates(bottom_right_x+rover.x_pos,-bottom_right_y+rover.y_pos)

        rover_border_left = bresenham(top_left_index[0],top_left_index[1],bottom_left_index[0],bottom_left_index[1])
        rover_border_top = bresenham(top_left_index[0],top_left_index[1],top_right_index[0],top_right_index[1])
        rover_border_bottom = bresenham(bottom_left_index[0],bottom_left_index[1],bottom_right_index[0],bottom_right_index[1])
        rover_border_right = bresenham(top_right_index[0],top_right_index[1],bottom_right_index[0],bottom_right_index[1])

        index_x_list = []
        index_y_list = []

        for l in rover_border_left:
            if self.is_occupied_idx((l[0],l[1])) == False:
                self.set_data_idx((l[0],l[1]), rover_region)
                index_x_list.append(l[0])
                index_y_list.append(l[1])
        for l in rover_border_top:
            if self.is_occupied_idx((l[0],l[1])) == False:
                self.set_data_idx((l[0],l[1]), rover_region)
                index_x_list.append(l[0])
                index_y_list.append(l[1])
        for l in rover_border_bottom:
            if self.is_occupied_idx((l[0],l[1])) == False:
                self.set_data_idx((l[0],l[1]), rover_region)
                index_x_list.append(l[0])
                index_y_list.append(l[1])
        for l in rover_border_right:
            if self.is_occupied_idx((l[0],l[1])) == False:
                self.set_data_idx((l[0],l[1]), rover_region)
                index_x_list.append(l[0])
                index_y_list.append(l[1])
        return index_x_list, index_y_list
    
    def set_rover_fill(self, index_x_list, index_y_list):
        # print(index_x_list)
        zipped_xy_list =  zip(index_x_list, index_y_list)
        sorted_index_xy_list = sorted(zipped_xy_list)
        sorted_index_line = {}

        for i in sorted_index_xy_list:
            if str(i[0]) in sorted_index_line:
                sorted_index_line[str(i[0])].append(i[1])
            else:
                list_y = [i[1]]
                sorted_index_line[str(i[0])] = list_y
        for i in sorted_index_line:
            list_y = sorted(sorted_index_line[i])
            sorted_index_line[i] = [list_y[0],list_y[-1]] # [min_y, max_y] for part. x coord
            # excluding the two extreme points where rover border is
            for j in range(list_y[-1]-list_y[0]-1):
                self.set_data_idx((int(i),sorted_index_line[i][0]+j+1), rover_region)


    def set_rover(self, rover):
        index_x_list, index_y_list = self.set_rover_borders_on_grid(rover)
        self.set_rover_fill(index_x_list, index_y_list)
    
    def plot_rover(self, rover):
        self.set_rover(rover)
        self.update_map()

    def update_rover(self, prev_rover, rover):
        self.remove_rover(prev_rover)
        self.set_rover(rover)
        self.update_map()

    def remove_rover_borders_on_grid(self, rover):
        top_left_ang = rover.ang_front_topleft  + rover.angle
        top_right_ang = rover.angle - rover.ang_front_topleft

        bottom_left_ang = top_left_ang + rover.ang_topleft_bottomleft
        bottom_right_ang = bottom_left_ang + rover.ang_back_to_bottomleft*2

        top_left_dist = rover.dist_centre_topcorners
        bottom_right_dist = rover.dist_centre_bottomcorners

        top_left_x = np.cos(math.radians(top_left_ang)) * top_left_dist
        top_left_y = np.sin(math.radians(top_left_ang)) * top_left_dist
        top_right_x = np.cos(math.radians(top_right_ang)) * top_left_dist
        top_right_y = np.sin(math.radians(top_right_ang)) * top_left_dist

        bottom_left_x = np.cos(math.radians(bottom_left_ang)) * bottom_right_dist
        bottom_left_y = np.sin(math.radians(bottom_left_ang)) * bottom_right_dist
        bottom_right_x = np.cos(math.radians(bottom_right_ang)) * bottom_right_dist
        bottom_right_y = np.sin(math.radians(bottom_right_ang)) * bottom_right_dist
        
        top_left_index = self.get_index_from_coordinates(top_left_x+rover.x_pos,-top_left_y+rover.y_pos)
        top_right_index = self.get_index_from_coordinates(top_right_x+rover.x_pos,-top_right_y+rover.y_pos)
        bottom_left_index = self.get_index_from_coordinates(bottom_left_x+rover.x_pos,-bottom_left_y+rover.y_pos)
        bottom_right_index = self.get_index_from_coordinates(bottom_right_x+rover.x_pos,-bottom_right_y+rover.y_pos)

        rover_border_left = bresenham(top_left_index[0],top_left_index[1],bottom_left_index[0],bottom_left_index[1])
        rover_border_top = bresenham(top_left_index[0],top_left_index[1],top_right_index[0],top_right_index[1])
        rover_border_bottom = bresenham(bottom_left_index[0],bottom_left_index[1],bottom_right_index[0],bottom_right_index[1])
        rover_border_right = bresenham(top_right_index[0],top_right_index[1],bottom_right_index[0],bottom_right_index[1])

        index_x_list = []
        index_y_list = []

        for l in rover_border_left:
            # dont need to check here whether grid cell was occupied previously
            # because was just previously occupied by rover
            self.set_data_idx((l[0],l[1]), known_empty_region)
            index_x_list.append(l[0])
            index_y_list.append(l[1])
        for l in rover_border_top:
            self.set_data_idx((l[0],l[1]), known_empty_region)
            index_x_list.append(l[0])
            index_y_list.append(l[1])
        for l in rover_border_bottom:
            self.set_data_idx((l[0],l[1]), known_empty_region)
            index_x_list.append(l[0])
            index_y_list.append(l[1])
        for l in rover_border_right:
            self.set_data_idx((l[0],l[1]), known_empty_region)
            index_x_list.append(l[0])
            index_y_list.append(l[1])
        return index_x_list, index_y_list

    def remove_rover_fill(self, index_x_list, index_y_list):
        # print(index_x_list)
        zipped_xy_list =  zip(index_x_list, index_y_list)
        sorted_index_xy_list = sorted(zipped_xy_list)
        sorted_index_line = {}

        for i in sorted_index_xy_list:
            if str(i[0]) in sorted_index_line:
                sorted_index_line[str(i[0])].append(i[1])
            else:
                list_y = [i[1]]
                sorted_index_line[str(i[0])] = list_y
        for i in sorted_index_line:
            list_y = sorted(sorted_index_line[i])
            sorted_index_line[i] = [list_y[0],list_y[-1]] # [min_y, max_y] for part. x coord
            # excluding the two extreme points where rover border is
            for j in range(list_y[-1]-list_y[0]-1):
                self.set_data_idx((int(i),sorted_index_line[i][0]+j+1), known_empty_region)

    def remove_rover(self, rover):
        index_x_list, index_y_list = self.remove_rover_borders_on_grid(rover)
        self.remove_rover_fill(index_x_list, index_y_list)



    def get_index_from_coordinates(self, x, y):
        """
        Get the array indices of the given point.
        :param x: the point's x-coordinate in meters
        :param y: the point's y-coordinate in meters
        :return: the corresponding array indices as a (x, y) tuple
        """
        x_index = int(round(round(x/self.cell_size,3)))
        y_index = int(round(round(y/self.cell_size,3)))

        return x_index, y_index

    def get_coordinates_from_index(self, x_index, y_index):
        """
        Get the coordinates of the given array point in meters.
        :param x_index: the point's x index
        :param y_index: the point's y index
        :return: the corresponding point in meters as a (x, y) tuple
        """
        x = x_index*self.cell_size
        y = y_index*self.cell_size

        return x, y

    def set_boundary(self, x_coord_list, y_coord_list, obj_color):
        boundary_xy_index_list = self.get_boundary_pts_index(x_coord_list, y_coord_list)
        self.set_boundary_pts_w_extrapolation(boundary_xy_index_list, obj_color)

    def get_boundary_pts_index(self, x_coord_list, y_coord_list):
        xy_index_w_bias_list = []
        for i in range(len(x_coord_list)):
            xy_index_w_bias = self.get_index_from_coordinates(x_coord_list[i]+self.bias,-y_coord_list[i]+self.bias)
            xy_index_w_bias_list.append(xy_index_w_bias)
        return xy_index_w_bias_list

    def set_boundary_pts_w_extrapolation(self, boundary_pts_index_list, obj_color):
        # extrapolation of boundary line between two neighbouring boundary points
        prev_ix = 0
        prev_iy = 0
        x_y_tuple = zip(boundary_pts_index_list, obj_color)

        for (ix, iy), color in x_y_tuple:
            if color==6:
                self.set_data_idx((ix,iy), known_boundary)
            elif color>=1 and color<=5:
                color_value = obj_value_grid[color]
                self.set_data_idx((ix,iy), color_value)
            if (prev_ix!=0 and prev_iy!=0):
                line_imag_boundary = bresenham(prev_ix, prev_iy, ix, iy)
                for l in line_imag_boundary:
                    # so that when line path overlaps with confirmed obstacles/boundaries - doesn't overwrite those values to  extrapolated boundaries value
                    if self.is_unknown_idx((l[0],l[1])) == True: 
                        self.set_data_idx((l[0],l[1]), extrapolated_boundary)
            prev_ix = ix
            prev_iy = iy

        close_line_imag_boundary = bresenham(prev_ix, prev_iy, boundary_pts_index_list[0][0], boundary_pts_index_list[0][1])
        for l in close_line_imag_boundary:
            # so that when line path overlaps with confirmed obstacles/boundaries - doesn't overwrite those values to extrapolated boundaries value
            if self.is_unknown_idx((l[0],l[1])) == True: 
                self.set_data_idx((l[0],l[1]), extrapolated_boundary)

    def flood_fill(self, rover):
        start_x_index, start_y_index = self.get_index_from_coordinates(rover.x_pos, rover.y_pos)
        # Fill empty areas with queue method
        sx, sy = self.dim_cells
        fringe = deque()
        fringe.appendleft((start_x_index, start_y_index))
        visited = {}
        while fringe:
            n = fringe.pop()
            nx, ny = n

            # West
            if nx > 0:
                if (((str(nx-1)+","+str(ny)) in visited) ==False):
                    if self.is_unknown_idx((nx-1, ny)) == True: 
                        self.set_data_idx((nx-1, ny), known_empty_region)
                        fringe.appendleft((nx-1, ny))
                        visited[str(nx-1)+","+str(ny)] = 'Visited'
            # East
            if nx < sx - 1:
                if ((str(nx+1)+","+str(ny)) in visited) ==False:
                    if self.is_unknown_idx((nx+1, ny)) == True: 
                        self.set_data_idx((nx+1, ny), known_empty_region)
                        fringe.appendleft((nx+1, ny))
                        visited[str(nx+1)+","+str(ny)] = 'Visited'
            # North
            if ny > 0:
                if ((str(nx)+","+str(ny-1)) in visited) ==False:
                    if self.is_unknown_idx((nx, ny-1)) == True: 
                        self.set_data_idx((nx, ny-1), known_empty_region)
                        fringe.appendleft((nx, ny - 1))
                        visited[str(nx)+","+str(ny-1)] = 'Visited'
            # South
            if ny < sy - 1:
                if ((str(nx)+","+str(ny+1)) in visited) ==False:
                    if self.is_unknown_idx((nx, ny+1)) == True: 
                        self.set_data_idx((nx, ny+1), known_empty_region)
                        fringe.appendleft((nx, ny + 1))
                        visited[str(nx)+","+str(ny+1)] = 'Visited'

    def target_obj_coord(self, rover, obj_coord_list, target_obj_color, bias):
        print("target_obj_color:", target_obj_color)
        obj_color_arr = np.array(obj_coord_list)[:,2]
        print("obj_color_arr:", obj_color_arr)
        print("obj_test:", obj_color_arr[1])
        target_obj_row = np.where(obj_color_arr==str(target_obj_color))
        # target color is not available
        print("target_obj_row:", target_obj_row)
        if target_obj_row==[]:
            print("Target object not found on current map")
        else:
            target_ox = float(obj_coord_list[target_obj_row[0][0]][0]) + bias
            target_oy = -float(obj_coord_list[target_obj_row[0][0]][1]) + bias

        print("target obj coord:", target_ox, target_oy)
        return target_ox, target_oy
        

    def plot_map(self):
        """
        helper function to plot two colormaps
        """
        top = cm.get_cmap('Greys', 128)
        bottom = cm.get_cmap('hsv', 128)

        newcolors = np.vstack((top(np.linspace(0, 1, 64)),
                            bottom(np.linspace(0, 1, 192))))
        newcmp = ListedColormap(newcolors, name='OrangeBlue')

        fig, axs = plt.subplots(figsize=(8,6))
        xydim = np.array(self.data).shape
        psm = axs.imshow(self.data, cmap=newcmp, rasterized=True, vmin=-4, vmax=4)
        psm.set_clim(0, 8)
        fig.colorbar(psm, ax=axs)
        plt.gca().set_xticks(np.arange(-.5, xydim[1], 1), minor = True)
        plt.gca().set_yticks(np.arange(-.5, xydim[0], 1), minor = True)
        plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)
        fig.tight_layout()
        plt.subplots_adjust(top=0.85)
        plt.pause(0.001)
        self.axs = psm
        self.fig = fig

    def update_map(self):
        if (self.axs == 0):
            raise Exception('AxesImage not set')
        self.axs.set_data(self.data)
        plt.draw()
        plt.pause(0.0001)