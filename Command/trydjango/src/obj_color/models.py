from django.db import models
from django.db.models.signals import post_save
from django.dispatch import receiver

from obj_color.tasks import path_finding_algo

# Create your models here.
class TargetObj(models.Model):
    id  = models.IntegerField(primary_key=True)
    color = models.CharField(max_length=10)
    status_done = models.IntegerField()

# Create your models here.
@receiver(post_save, sender=TargetObj)
def check_update_in_color(sender, **kwargs):
    # if database was created or updated
    db_obj_color = kwargs.get('instance')
    if db_obj_color.status_done==0:
        # to calculate path from rover to obj
        # trigger a background task to calculate
        target_obj = TargetObj.objects.get(id=1)
        target_obj_color = target_obj.color
        print("object_color:", target_obj_color)
        path_finding_algo.apply_async(args = [target_obj_color])
