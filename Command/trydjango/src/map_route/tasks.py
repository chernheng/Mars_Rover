from celery import shared_task
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer

import pickle
import io
import base64
import csv
import math

from map_route.load_map import get_initial_map_array
from map_route.models import GmapArray
from rover_obj.models import Rover_obj
from map_route.occupancyGrid import OccupancyGridMap

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
def run_map_algo():
    print("running run_map_algo")
    map_data_arr, rover_obj, gmap, obj_coord_list, rover_start_coord = get_initial_map_array()
    #saving obj coordinates into csv file
    with open('map_route/obj_coordinates.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(obj_coord_list)

    # serialisation of numpy array data
    np_bytes = pickle.dumps(map_data_arr)
    np_base64 = base64.b64encode(np_bytes)
    gmap_obj_db, gmap_created = GmapArray.objects.update_or_create(
        id=1,
        defaults={'id': 1, 'data': np_base64}
    )
    rover_obj_db, created = Rover_obj.objects.update_or_create(
        id=1,
        defaults = {
            'id': 1,
            'length': rover_obj.length,
            'width': rover_obj.width,
            'x_pos': rover_obj.x_pos,
            'y_pos': rover_obj.y_pos,
            'angle': rover_obj.angle,
            'pivot_length': rover_obj.pivot_length,
            'dist_centre_topcorners': rover_obj.dist_centre_topcorners,
            'dist_centre_bottomcorners': rover_obj.dist_centre_bottomcorners,
            'ang_topcorner_bottomcorner': rover_obj.ang_topcorner_bottomcorner,
            'ang_front_topleft': rover_obj.ang_front_topleft,
            'ang_back_to_bottomleft': rover_obj.ang_back_to_bottomleft,
            'ang_topleft_bottomleft': rover_obj.ang_topleft_bottomleft,
        }
        
    )
    map_size = 100
    xyreso = 5  # x-y grid resolution
    bias = math.floor(0.5*map_size*xyreso)
    gmap = OccupancyGridMap(map_data_arr, xyreso, bias)
    # set rover
    gmap.set_rover(rover_obj)
    gmap.plot_map()

    # serialisation of numpy array data
    np_bytes_update = pickle.dumps(gmap.data)
    np_base64_update = base64.b64encode(np_bytes_update)
    GmapArray.objects.update_or_create(
        id=1,
        defaults={
            'id': 1,
            'data': np_base64
        }
    )

    update_websocket(gmap.fig)

def update_websocket(map_fig):
    print("running update_websocket")
    flike = io.BytesIO()
    map_fig.savefig(flike, bbox_inches='tight',pad_inches = 0)
    # converts binary/bytes to base64 string
    b64_str = base64.b64encode(flike.getvalue()).decode('utf-8')

    channel_layer = get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        "map",
        {
            "type": "map.message",
            "text": b64_str,
        },
    )
    flike.close()