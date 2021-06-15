import math
from heapq import heappush, heappop

def dist2d(point1, point2):
    # euclidean distance between the two points
    x1, y1 = point1[0:2]
    x2, y2 = point2[0:2]

    dist2 = (x1 - x2)**2 + (y1 - y2)**2

    return math.sqrt(dist2)

def _get_movements_8n():
    # Get all possible movements in the 8 neighbouring directions
    # returns list of movements with cost [(dx, dy, movement_cost, angle)]
    s2 = math.sqrt(2)
    return [(1, 0, 1.0, 0),
            (0, 1, 1.0, 270),
            (-1, 0, 1.0, 180),
            (0, -1, 1.0, 90),
            (1, 1, s2, 315),
            (-1, 1, s2, 225),
            (-1, -1, s2, 135),
            (1, -1, s2, 45)]

def a_star(start_m, goal_m, gmap, rover, movement='8N'):
    # A* algorithm for 2D occupancy grid map
    # start_m: start node (x, y) in cm
    # goal_m: goal node (x, y) in cm
    # gmap: the grid map object
    # returns a tuple that contains: (generated path list in terms of coords, the resulting path list in terms of array index, rover's ending position)

    print("Running a star algo...")
    start = gmap.get_index_from_coordinates(start_m[0], start_m[1])
    goal = gmap.get_index_from_coordinates(goal_m[0], goal_m[1])
    print("Start index:", start)
    print("Goal index:", goal)

    # check if starting node correspond to free space
    if gmap.is_occupied_idx(start):
        raise Exception('Start node is not traversable')

    # add start node to front
    # front is a list of (total estimated cost to goal, total cost from start to node, node, previous node)
    start_node_cost = 0
    start_node_estimated_cost_to_goal = dist2d(start, goal) + start_node_cost
    front = [(start_node_estimated_cost_to_goal, start_node_cost, start, None, dist2d(start, goal), (start_m[0], start_m[1]))]

    # use a dictionary to remember where we came from in order to reconstruct the path later on
    came_from = {}

    # # get possible movements
    if movement == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    # while there are elements to investigate in our front.
    while front:
        # get smallest item and remove from front.
        element = heappop(front)
        
        # if this has been visited already, skip it
        # in terms of index
        total_cost, cost, pos, previous, dist_from_goal, pos_coord = element

        if gmap.is_visited_idx(pos):
            print("Already visited")
            continue

        # # now it has been visited, mark with cost
        gmap.mark_visited_idx(pos)

        # set its previous node
        came_from[pos] = previous

        # gives leeway such that object is close enough to the object yet is far enough to be seen by the camera
        if dist_from_goal<=(35+15)/gmap.cell_size:
            print("dist_from_goal:", dist_from_goal)
            break

        # check all neighbors
        for dx, dy, deltacost, angle in movements:
            # determine new position
            new_x = pos[0] + dx
            new_y = pos[1] + dy
            new_pos = (new_x, new_y)
            new_coord_x = pos_coord[0] + dx*gmap.cell_size
            new_coord_y = pos_coord[1] + dy*gmap.cell_size
            new_coord_pos = (new_coord_x, new_coord_y)

            rover.angle = angle
            rover.x_pos = new_coord_x
            rover.y_pos = new_coord_y
            # check whether new position is inside the map
            # if not, skip node
            if not gmap.is_inside_idx(new_pos):
                continue

            # add node to front if it was not visited before and is not an obstacle
            if (not gmap.is_visited_idx(new_pos)) and (not gmap.rover_collision_idx(rover)):
                new_cost = cost + deltacost
                new_total_cost_to_goal = new_cost + dist2d(new_pos, goal) 
                new_dist_from_goal = dist2d(new_pos, goal)

                heappush(front, (new_total_cost_to_goal, new_cost, new_pos, pos, new_dist_from_goal, new_coord_pos))
                print("Pushed to heap, pos", new_pos)

    # reconstruct path backwards (only if we reached the goal)
    path = []
    path_idx = []
    while pos:
        print("pos path:", pos)
        path_idx.append(pos)
        # transform array indices to coordinates
        pos_m_x, pos_m_y = gmap.get_coordinates_from_index(pos[0], pos[1])
        path.append((pos_m_x, pos_m_y))
        pos = came_from[pos]

    rover_end_pos = (path_idx[0][0], path_idx[0][1])

    # reverse so that path is from start to goal.
    path.reverse()
    path_idx.reverse()

    return path, path_idx, rover_end_pos