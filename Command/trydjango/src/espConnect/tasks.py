from celery import shared_task

import numpy as np
import csv
import pickle
import io
import base64
import math
import time

from instruction.models import Instruction
from command.models import Command
from rover_obj.models import Rover_obj
from map_route.models import GmapArray
from map_route.occupancyGrid import OccupancyGridMap
from obj_color.utils import get_rover_angle, update_websocket
from map_route.rover import Rover
from obj_color.models import TargetObj
# from espConnect.models import Rover_actual
from map_route.utils import get_index_from_coord
from obj_color.tasks import path_finding_algo

@shared_task
def update_command_db(updated_ang, updated_dist):
    xyreso = 5
    # get the instructions that are not yet executed
    first_filtered_instr = Instruction.objects.filter(sub_instr_done=0).first()
    num_instr_not_done = Instruction.objects.filter(sub_instr_done=0).count()
    print("first unexecuted instruction:", first_filtered_instr)
    print("printing db obj instruction:", Instruction.objects.get(id=0))
    print("Remaining number of instructions:", num_instr_not_done)
    
    # getting gmap data
    map_size = 100
    xyreso = 5  # x-y grid resolution
    rover_start_coord  = float(math.floor(0.5*map_size*xyreso))
    bias = math.floor(0.5*map_size*xyreso)
    map_obj = GmapArray.objects.get(id=1)
    np_bytes = base64.b64decode(map_obj.data)
    map_data_arr = pickle.loads(np_bytes)
    gmap = OccupancyGridMap(map_data_arr, xyreso, bias)
    gmap.plot_map()

    # reflect actual movement to map display
    rover_obj_db = Rover_obj.objects.get(id=1)
    prev_rover_obj = Rover(rover_obj_db.length, rover_obj_db.width, rover_obj_db.x_pos, rover_obj_db.y_pos, rover_obj_db.pivot_length, rover_obj_db.angle)
    prev_index = get_index_from_coord(gmap, rover_obj_db.x_pos, rover_obj_db.y_pos)
    current_rover_obj = prev_rover_obj
    if first_filtered_instr.angle != updated_ang:
        print("Actual rover's angle differs from that on the map")
        # positive ang_err means under rotation
        ang_err = first_filtered_instr.angle - updated_ang
        ang_unwrapped = rover_obj_db.angle + ang_err
        if ang_unwrapped<0:
            ang_unwrapped = ang_unwrapped + 360
        elif ang_unwrapped>360:
            ang_unwrapped = ang_unwrapped - 360
        rover_obj_db.angle = ang_unwrapped
        rover_obj_db.save()
        current_rover_obj = Rover(rover_obj_db.length, rover_obj_db.width, rover_obj_db.x_pos, rover_obj_db.y_pos, rover_obj_db.pivot_length, rover_obj_db.angle)
        if ang_err>2:
            print("Correcting rover's angle on map")
            gmap.update_rover(prev_rover_obj, current_rover_obj)
            gmap_data_bytes = pickle.dumps(gmap.data)
            gmap_data_b64 = base64.b64encode(gmap_data_bytes)
            map_obj_update = GmapArray.objects.get(id=1)
            map_obj_update.data = gmap_data_b64
            map_obj_update.save()

            update_websocket(gmap.fig)
            print("Updating gmap db")
            obj_db = TargetObj.objects.get(id=1)
            print("Recalculating shortest path to object...")
            path_finding_algo(obj_db.color)
            return True
    
    if first_filtered_instr.distance != updated_dist:
        print("Actual rover coord position differs from current map")
        # postive coord_err means rover doesnt move enough
        coord_err = first_filtered_instr.distance - updated_dist
        ang = rover_obj_db.angle + 180
        if ang>360:
            ang = ang -360
        delta_y = math.sin(math.radians(ang)) * coord_err
        delta_x = math.cos(math.radians(ang)) * coord_err
        new_x_pos = rover_obj_db.x_pos + delta_x
        new_y_pos = rover_obj_db.y_pos - delta_y
        rover_obj_db.x_pos = new_x_pos
        rover_obj_db.y_pos = new_y_pos
        rover_obj_db.save()
        current_rover_obj = Rover(rover_obj_db.length, rover_obj_db.width, rover_obj_db.x_pos, rover_obj_db.y_pos, rover_obj_db.pivot_length, rover_obj_db.angle)

        new_index = get_index_from_coord(gmap, new_x_pos, new_y_pos)
        # update the map if the rover's position is not the same as predicted
        if new_index!=prev_index:
            print("Actual rover grid position differs from current map")
            print("angle of rover:", current_rover_obj.angle)
            gmap.update_rover(prev_rover_obj, current_rover_obj)
            gmap_data_bytes = pickle.dumps(gmap.data)
            gmap_data_b64 = base64.b64encode(gmap_data_bytes)
            map_obj_update = GmapArray.objects.get(id=1)
            map_obj_update.data = gmap_data_b64
            map_obj_update.save()

            update_websocket(gmap.fig)
            print("Updating gmap db")
            obj_db = TargetObj.objects.get(id=1)
            print("Recalculating shortest path to object...")
            path_finding_algo(obj_db.color)
            return True
    # set this instruction as done
    first_filtered_instr.sub_instr_done = 1
    first_filtered_instr.save()

    num_instr_not_done = num_instr_not_done - 1
    # means done executing all instructions
    if num_instr_not_done==0:
        target_obj_db = TargetObj.objects.get(id=1)
        target_obj_db.status_done = 1
        target_obj_db.save()

    second_filtered_instr = Instruction.objects.filter(sub_instr_done=0).first()
    # if there are still subinstructions left that are undone
    if num_instr_not_done!= 0:
        # update the command db to be the next sub_instr
        instr_id = second_filtered_instr.id
        ang = second_filtered_instr.angle
        dist = second_filtered_instr.distance
        last_path = second_filtered_instr.last_path
        command_update = Command.objects.get(id=1)
        command_update.instr_tag = command_update.instr_tag + 1
        command_update.angle = ang
        command_update.distance = dist
        command_update.last_path = last_path
        command_update.save()

        # run animation for this sub_instr
        current_sub_instr_obj = first_filtered_instr
        prev_path_index_stop = current_sub_instr_obj.end_index
        next_sub_instr_obj = second_filtered_instr
        path_index_stop = next_sub_instr_obj.end_index

        with open('obj_color/path_px_list.csv', newline='') as f:
            reader = csv.reader(f)
            path_px = list(reader)

        with open('obj_color/path_list.csv', newline='') as f:
            reader = csv.reader(f)
            path = list(reader)

        rover_obj_db = Rover_obj.objects.get(id=1)
        rover_obj = Rover(25,20,rover_obj_db.x_pos, rover_obj_db.y_pos,15,rover_obj_db.angle)
        # array of strings - because read from csv file
        path_px_arr = np.array(path_px)
        # array of strings - because read from csv file
        path_arr = np.array(path)
        prev_angle = rover_obj.angle
        
        # if the instr is a angle rotation
        if path_index_stop==prev_path_index_stop:
            prev_rover_obj = Rover(
                    rover_obj.length, 
                    rover_obj.width, 
                    rover_obj.x_pos, 
                    rover_obj.y_pos, 
                    rover_obj.pivot_length, 
                    rover_obj.angle
                )
            rover_obj_update_db = Rover_obj.objects.get(id=1)
            # this is the relative angle
            current_angle = second_filtered_instr.angle
            raw_ang = prev_angle - current_angle
            if raw_ang<0:
                raw_ang = raw_ang + 360
            elif raw_ang>=360:
                raw_ang = raw_ang-360
            rover_obj_update_db.angle = raw_ang
            rover_obj_update_db.save()

            print("new angle:", rover_obj_update_db.angle)
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

            update_websocket(gmap.fig)
            print("Updating gmap db")
        else:
            for j in range(path_index_stop-prev_path_index_stop):
                prev_rover_obj = Rover(
                    rover_obj.length, 
                    rover_obj.width, 
                    rover_obj.x_pos, 
                    rover_obj.y_pos, 
                    rover_obj.pivot_length, 
                    rover_obj.angle
                )
                print("prev_rover_obj y_pos:", prev_rover_obj.y_pos)
                rover_obj.x_pos = float(path_arr[j+prev_path_index_stop+1][0])
                rover_obj.y_pos = float(path_arr[j+prev_path_index_stop+1][1])
                print("rover_obj y_pos:", rover_obj.y_pos)
                print("checking prev_rover_obj y_pos:", prev_rover_obj.y_pos)

                rover_obj_update_db = Rover_obj.objects.get(id=1)
                rover_obj_update_db.x_pos = rover_obj.x_pos
                rover_obj_update_db.y_pos = rover_obj.y_pos
                rover_obj_update_db.save()


                gmap.update_rover(prev_rover_obj, rover_obj)
                gmap_data_bytes = pickle.dumps(gmap.data)
                gmap_data_b64 = base64.b64encode(gmap_data_bytes)
                map_obj_update = GmapArray.objects.get(id=1)
                map_obj_update.data = gmap_data_b64
                map_obj_update.save()

                update_websocket(gmap.fig)
                print("Updating gmap db")



