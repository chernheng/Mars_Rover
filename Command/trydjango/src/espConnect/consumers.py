import json
from asgiref.sync import async_to_sync
from channels.generic.websocket import WebsocketConsumer
from csv import writer

from .models import Rover_actual
from spin.models import Spin
from command.models import Command

class EspConnectConsumer(WebsocketConsumer):
    def connect(self):
        async_to_sync(self.channel_layer.group_add)("rover_update", self.channel_name)
        obj, created = Command.objects.get_or_create(
            id=1, 
            defaults={
                'id': 1,
                'instr_tag': 0, 
                'angle': 0, 
                'distance': 0, 
                'obj_color': 0,
                'speed': 255,
                'last_path': 0
            }
        )
        obj_2, created_2 = Spin.objects.get_or_create(
            id=1,
            defaults={'id': 1, 'start': 0, 'end': 0},
        )
        self.accept()

    def disconnect(self, close_code):
        async_to_sync(self.channel_layer.group_discard)("rover_update", self.channel_name)

    # Receive message from WebSocket - obstacle points or actual rover's position sent from esp
    def receive(self, text_data):
        text_data_json = json.loads(text_data)
        message = text_data_json['text']

        print("Received data from espconnect websocket")
        print("Received data", message)

        # rover_status = json.dumps(message)
        if 'done' in message:
            if message['done']==1:
                end_spin = Spin.objects.get(id=1)
                end_spin.end = 1
                end_spin.save()
        elif 'start' in message:
            rover_status = json.loads(message)
            async_to_sync(self.channel_layer.group_send)(
                "rover_update",
                {
                    "type": "rover.message",
                    "text": rover_status,
                },
            )
        elif 'angle' in message:
            ang = message["angle"]
            dist = message["distance"]
            obj_color = message["obj_color"]
            # updates the database or create a new object of the Rover class
            # creates the new object when the first message is received
            # updates database regardless Phase 1 or Phase 2
            obj, created = Rover_actual.objects.update_or_create(
                id = 1,
                defaults={
                    'id': 1, 
                    'obj_color': obj_color,
                    'angle': ang,
                    'distance': dist,
                    'battery': message["battery"],
                    'range_estimate': message["range_estimate"]
                },
            )
            spin_db = Spin.objects.get(id=1)
            # still in the midst of Phase 1 - append data points of obstacle/objects to storage file
            if spin_db.end == 0 and spin_db.start==1:
                append_list_as_row('map_route/map_data.csv', (ang, dist, obj_color))
        else:
            print("Received type of rover's status not supported")
        
    # Receive message from web server - sends message of new instruction when theres an update in command db
    def rover_message(self, event):
        print("received messages to update command (espConnect)")

        self.send(text_data=json.dumps({
            "type": "websocket.send",
            "text": event["text"],
        }))



def append_list_as_row(file_name, list_of_elem):
    # Open file in append mode
    with open(file_name, 'a+', newline='') as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj)
        # Add contents of list as last row in the csv file
        csv_writer.writerow(list_of_elem)