from django.db import models
from django.db.models.signals import post_save
from django.dispatch import receiver
from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer

from instruction.models import Instruction
from espConnect.tasks import update_command_db



# Create your models here.
# we want backend to have memory of the rover's current condition (storage)

class Rover_actual(models.Model):
    id              = models.IntegerField(primary_key=True)
    obj_color       = models.IntegerField() # 0: invalid, 1-5: colour, 6:dist to wall/boundary (if 0: means too close to wall to see bottom of wall)
    angle           = models.IntegerField()
    distance        = models.IntegerField()
    battery         = models.PositiveSmallIntegerField()
    range_estimate  = models.IntegerField()

# Create your models here.
@receiver(post_save, sender=Rover_actual)
def check_actual_rover_status(sender, **kwargs):
    update_rover_position_db = kwargs.get('instance')
    print("Change in rover_actual db")
    # obj_color is 0 if sending rover's position 
    # if this is actual navigation phase, there will be instructions in the db
    if Instruction.objects.count()>0 and update_rover_position_db.obj_color==0:
        print("Instructions detected in db")
        update_command_db.apply_async([update_rover_position_db.angle, update_rover_position_db.distance])
    
    # update battery percentage
    new_batt = update_rover_position_db.battery
    new_batt_update_str = "battery: "+str(new_batt)+"%"

    channel_layer = get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        "map",
        {
            "type": "map.message",
            "text": new_batt_update_str,
        },
    )
    if new_batt_update_str == "15%":
        # 15% battery triggers low power mode
        channel_layer = get_channel_layer()
        async_to_sync(channel_layer.group_send)(
            "map",
            {
                "type": "map.message",
                "text": "critical_power",
            },
        )
        