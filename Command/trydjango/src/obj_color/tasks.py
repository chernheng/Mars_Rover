from celery import shared_task

from rover_obj.models import Rover_obj
from instruction.models import Instruction

import pickle
import base64
import numpy as np
import math
import csv

from map_route.load_map import get_initial_map_array
from map_route.a_star import a_star
from map_route.models import GmapArray
from map_route.rover import Rover
from map_route.occupancyGrid import OccupancyGridMap
from map_route.utils import get_rover_command
from command.models import Command
from obj_color.utils import get_rover_angle, update_websocket, plot_map

# grid map values
unknown_region = 1
known_empty_region = 0
known_boundary = 1.9
extrapolated_boundary = 1.5
rover_region = 1.9
color_to_index = {"red": 1,
                    "green": 2,
                    "blue": 3,
                    "yellow": 4,
                    "purple": 5
                }
obj_value_grid = [0,8,4,6,3.1,6.7]

@shared_task
def path_finding_algo(target_obj_color):
    print("running path_finding_algo")
    target_obj_color_index = color_to_index[target_obj_color]
    print("color_index:", target_obj_color_index)
    target_occ_value = obj_value_grid[target_obj_color_index]

    map_size = 100
    xyreso = 5  # x-y grid resolution
    rover_start_coord  = float(math.floor(0.5*map_size*xyreso))
    # rover obj row created in map_route/tasks for default value when color is input for the first time
    rover_obj_db = Rover_obj.objects.get(id=1)
    rover_obj = Rover(rover_obj_db.length,rover_obj_db.width,rover_obj_db.x_pos, rover_obj_db.y_pos,rover_obj_db.pivot_length,rover_obj_db.angle)

    bias = math.floor(0.5*map_size*xyreso)
    map_obj_db = GmapArray.objects.get(id=1)
    np_bytes = base64.b64decode(map_obj_db.data)
    map_data_arr = pickle.loads(np_bytes)
    gmap = OccupancyGridMap(map_data_arr, xyreso, bias)
    gmap.plot_map()
    rover_obj_tmp = Rover(25,20,rover_obj_db.x_pos, rover_obj_db.y_pos,15,rover_obj_db.angle)
    gmap.remove_rover(rover_obj_tmp)

    with open('map_route/obj_coordinates.csv', newline='') as f:
        reader = csv.reader(f)
        obj_coord_list = list(reader)

    print("obj_coord_list:", obj_coord_list)

    start_node = (rover_obj.x_pos, rover_obj.y_pos)
    goal_node = gmap.target_obj_coord(rover_obj, obj_coord_list, target_obj_color_index, bias)

    # run A* algo to find path
    path, path_px, rover_end_pos = a_star(start_node, goal_node, gmap, rover_obj, movement='8N')

    print("path_px from output of a star:", path_px)
    # write path_px_list to csv file
    with open('obj_color/path_px_list.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(path_px)

    with open('obj_color/path_list.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(path)
    

    # reset rover obj class to starting position - because it may have been updated in a star algo to calculate path
    rover_obj = Rover(25,20,rover_obj_db.x_pos, rover_obj_db.y_pos,15,rover_obj_db.angle)
    if path_px:
        # [relative_angle, distance, last_index]
        instr_list = get_rover_command(gmap, path_px, rover_obj, goal_node, rover_end_pos)
        
        # reset rover_obj
        rover_obj = Rover(25,20,rover_obj_db.x_pos, rover_obj_db.y_pos,15,rover_obj_db.angle)
        print("rover's angle after get_rover_command", rover_obj.angle)
        # print(instr_list)
        instr_len = len(instr_list)
        instr_track = 0

        # extra check - should have been empty/deleted when user called CLEAR
        # delete past list of instructions if another target obj color was entered after the first one
        if Instruction.objects.count()>0:
            Instruction.objects.all().delete()
        # store to database list of instructions to get to obj color
        for i in range(instr_len):
            print("instr_list:", instr_list[i])
            is_last_path=0
            # second last path
            if i == instr_len-2:
                is_last_path = 1
            # last path
            if i == instr_len-1:
                is_last_path = 2

            Instruction.objects.create(
                id=instr_track, 
                angle = instr_list[i][0], 
                distance = instr_list[i][1], 
                end_index = instr_list[i][2], 
                last_path = is_last_path, 
                sub_instr_done = 0
            )
            instr_track = instr_track + 1
        
        command_last_path = 0
        if instr_len==2:
            command_last_path = 1
        if instr_len==1:
            command_last_path = 2
        # update command with the first path
        Command.objects.update_or_create(
            id=1, 
            defaults={
                'id': 1,
                'instr_tag': 1, 
                'angle': instr_list[0][0], 
                'distance': instr_list[0][1], 
                'obj_color': target_obj_color_index,
                'last_path': command_last_path
            }
        )

        # just plot animation for first path
        path_index_stop = instr_list[0][2]

        path_px_arr = np.array(path_px)
        path_arr = np.array(path)
        prev_angle = rover_obj.angle
        print("starting rover's angle:", prev_angle)
        prev_x = rover_obj.x_pos
        prev_y = rover_obj.y_pos
        # if the first instr is a angle rotation
        if path_index_stop==0:
            # prev_rover_obj = rover_obj
            prev_rover_obj = Rover(
                    rover_obj.length, 
                    rover_obj.width, 
                    rover_obj.x_pos, 
                    rover_obj.y_pos, 
                    rover_obj.pivot_length, 
                    rover_obj.angle
                )
            print("prev_rover_obj x_pos:", prev_rover_obj.x_pos)
            print("rover_obj x_pos:", rover_obj.x_pos)

            rover_obj_update_db = Rover_obj.objects.get(id=1)
            current_angle = rover_obj.angle - Instruction.objects.get(id=0).angle
            if current_angle<0:
                current_angle = current_angle + 360
            rover_obj_update_db.angle = current_angle
            rover_obj_update_db.save()

            current_rover_obj = Rover(
                length = rover_obj_update_db.length,
                width  = rover_obj_update_db.width,
                x_pos = rover_obj_update_db.x_pos,
                y_pos = rover_obj_update_db.y_pos,
                pivot_length = rover_obj_update_db.pivot_length,
                angle = rover_obj_update_db.angle
            )
            gmap.update_rover(prev_rover_obj, current_rover_obj)
            gmap_data_bytes = pickle.dumps(gmap.data)
            gmap_data_b64 = base64.b64encode(gmap_data_bytes)
            map_obj_update = GmapArray.objects.get(id=1)
            map_obj_update.data = gmap_data_b64
            map_obj_update.save()

            gmap.update_map()

            update_websocket(gmap.fig)
            print("Updating gmap db")
        else:
            
            for j in range(path_index_stop):
                prev_rover_obj = Rover(
                    rover_obj.length, 
                    rover_obj.width, 
                    rover_obj.x_pos, 
                    rover_obj.y_pos, 
                    rover_obj.pivot_length, 
                    rover_obj.angle
                )
                print("prev_rover_obj y_pos:", prev_rover_obj.y_pos)
                rover_obj.x_pos = path_arr[j+1][0]
                rover_obj.y_pos = path_arr[j+1][1]
                print("rover_obj y_pos:", rover_obj.y_pos)
                print("checking prev_rover_obj y_pos:", prev_rover_obj.y_pos)

                rover_obj_update_db = Rover_obj.objects.get(id=1)
                rover_obj_update_db.x_pos = rover_obj.x_pos
                rover_obj_update_db.y_pos = rover_obj.y_pos
                current_angle = get_rover_angle(prev_angle, rover_obj.x_pos, rover_obj.y_pos, prev_x, prev_y)
                rover_obj_update_db.angle = current_angle
                rover_obj_update_db.save()


                gmap.update_rover(prev_rover_obj, rover_obj)
                gmap_data_bytes = pickle.dumps(gmap.data)
                gmap_data_b64 = base64.b64encode(gmap_data_bytes)
                map_obj_update = GmapArray.objects.get(id=1)
                map_obj_update.data = gmap_data_b64
                map_obj_update.save()

                gmap.update_map()

                update_websocket(gmap.fig)
                print("Updating gmap db")

                prev_x = path_arr[j+1][0]
                prev_y = path_arr[j+1][1]
                prev_angle = current_angle
    else:
        print('Goal is not reachable')

