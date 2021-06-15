from django.db import models

# Create your models here.
class Instruction(models.Model):
    id        = models.IntegerField(primary_key=True)
    angle     = models.IntegerField()
    distance  = models.IntegerField()
    end_index = models.IntegerField()
    last_path = models.IntegerField()
    sub_instr_done = models.IntegerField() # 1: done