from django.contrib import admin

# Register your models here.
from .models import Instruction

admin.site.register(Instruction)