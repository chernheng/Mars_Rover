from django.db import models

class GmapArray(models.Model):
    id = models.IntegerField(primary_key=True)
    data = models.BinaryField()