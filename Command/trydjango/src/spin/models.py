from django.db import models
from django.dispatch import receiver
from django.db.models.signals import post_save

from map_route.tasks import run_map_algo

class Spin(models.Model):
    id      = models.IntegerField(primary_key=True)
    start   = models.IntegerField()
    end     = models.IntegerField()

# grid map values
unknown_region = 1
known_empty_region = 0
known_boundary = 1.9
extrapolated_boundary = 1.5
rover_region = 1.9

@receiver(post_save, sender=Spin)
def check_end_of_spin(sender, **kwargs):
    # if database is updated
    spin_db = Spin.objects.get(id=1)
    # when reset or just started - can be creating row or updating row
    if spin_db.end == 0 and spin_db.start==0:
            fileVariable = open('map_route/map_data.csv', 'r+')
            fileVariable.truncate(0)
            fileVariable.close()

    if kwargs.get('created')==False:
        if spin_db.end==1:
            # update gmap array in database with boundary pts?
            print("Detected change in Spin dataase END field")
            run_map_algo.apply_async()