import json
from asgiref.sync import async_to_sync
from channels.generic.websocket import WebsocketConsumer

from spin.models import Spin
from obj_color.models import TargetObj
from instruction.models import Instruction
from command.models import Command
from espConnect.models import Rover_actual
from map_route.models import GmapArray
from rover_obj.models import Rover_obj
from map_route.occupancyGrid import OccupancyGridMap
from obj_color.utils import update_websocket

import math
import base64
import pickle

class MapRouteConsumer(WebsocketConsumer):
    def connect(self):
        async_to_sync(self.channel_layer.group_add)("map", self.channel_name)
        if Spin.start==1 and Spin.end==1:
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

            update_websocket(gmap.fig)
            print("Updating gmap db")
        
        self.accept()

    def disconnect(self, close_code):
        async_to_sync(self.channel_layer.group_discard)("map", self.channel_name)

    # Receive message from WebSocket
    def receive(self, text_data):
        text_data_json = json.loads(text_data)
        message = text_data_json['text']

        if message=="start":
            start_spin_obj = Spin.objects.get(id=1)
            start_spin_obj.start = 1
            start_spin_obj.save()
        elif message == "clear":
            Instruction.objects.all().delete()
            Command.objects.all().delete()
            Rover_actual.objects.all().delete()
            GmapArray.objects.all().delete()
            TargetObj.objects.all().delete()
            Rover_obj.objects.all().delete()
            Spin.objects.all().delete()
        elif message == "low_speed":
            command_speed_db = Command.objects.get(id=1)
            command_speed_db.speed = 255
            command_speed_db.save()
        elif message == "med_speed":
            command_speed_db = Command.objects.get(id=1)
            command_speed_db.speed = 500
            command_speed_db.save()
        elif message == "high_speed":
            command_speed_db = Command.objects.get(id=1)
            command_speed_db.speed = 700
            command_speed_db.save()
        else:
            try:
                target_color_obj = TargetObj.objects.get(id=1)
                if target_color_obj.status_done == 1:
                    target_color_obj.color = message
                    target_color_obj.status_done = 0
                    target_color_obj.save()
                    print("Saved new target color to db")
            except TargetObj.DoesNotExist:
                target_color_obj = TargetObj.objects.create(
                                        id=1, 
                                        color = message, 
                                        status_done = 0
                                    )
                print("Created new target color in db")
            print("Detected color input from user")
         

    # Receive message from room group
    def map_message(self, event):
        print("received messages to update map data")
        
        self.send(text_data=json.dumps({
            "type": "websocket.send",
            "text": event["text"],
        }))