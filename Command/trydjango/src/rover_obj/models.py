from django.db import models

# Create your models here.
class Rover_obj(models.Model):
    id      = models.IntegerField(primary_key=True)
    length  = models.IntegerField()
    width   = models.IntegerField()
    x_pos   = models.FloatField()
    y_pos   = models.FloatField()
    angle   = models.FloatField()
    pivot_length = models.IntegerField()
    dist_centre_topcorners = models.FloatField()
    dist_centre_bottomcorners = models.FloatField()
    ang_topcorner_bottomcorner = models.FloatField()
    ang_front_topleft = models.FloatField()
    ang_back_to_bottomleft = models.FloatField()
    ang_topleft_bottomleft = models.FloatField()