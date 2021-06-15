from django.db import models
from django.db.models.signals import post_save
from django.dispatch import receiver
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer

# Create your models here.
class Command(models.Model):
    #attributes - to map to database
    id           = models.IntegerField(primary_key=True)
    instr_tag    = models.IntegerField()
    angle        = models.IntegerField()
    distance     = models.IntegerField()
    obj_color    = models.IntegerField()
    speed        = models.IntegerField() # low:255, med:500, high:700
    last_path    = models.IntegerField() # 2 for last path, 1 for second last path

# Create your models here.
@receiver(post_save, sender=Command)
def update_websocket(sender, **kwargs):
    # when command database is updated - these are actual commands, not just initialising db
    if kwargs.get('created')==False:
        new_rover_command = Command.objects.get(id=1)
        status_dict = {
            'instr_tag': new_rover_command.instr_tag, 
            'angle': new_rover_command.angle, 
            'distance': new_rover_command.distance, 
            'obj_color': new_rover_command.obj_color,
            'speed': new_rover_command.speed,
            'last_path': new_rover_command.last_path
            }
        channel_layer = get_channel_layer()
        async_to_sync(channel_layer.group_send)(
            "rover_update",
            {
                "type": "rover.message",
                "text": status_dict,
            },
        )