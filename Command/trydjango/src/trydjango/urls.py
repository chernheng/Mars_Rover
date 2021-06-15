"""trydjango URL Configuration

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/3.0/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
    # Use the following instead for greater clarity - because you might want to import 2 different views from different apps - which view does it refer to then?
    1. from pages.views import home_view
    2. path('', home_view, name='home'),
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""
from django.contrib import admin
from django.urls import path

from pages.views import home_view
from map_route.views import map_route_view

urlpatterns = [
    path('', home_view, name='home'),
    path('map_route/', map_route_view),
    path('admin/', admin.site.urls),
]
