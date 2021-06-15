"""
ASGI config for trydjango project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/3.0/howto/deployment/asgi/
"""

import espConnect
import os
import django

from channels.routing import ProtocolTypeRouter, URLRouter
from django.core.asgi import get_asgi_application
from channels.auth import AuthMiddlewareStack
from django.urls import re_path

from map_route.consumers import MapRouteConsumer
from espConnect.consumers import EspConnectConsumer

os.environ.setdefault('DJANGO_SETTINGS_MODULE', 'trydjango.settings')
django.setup()

application = ProtocolTypeRouter({
    "http": get_asgi_application(),
    "websocket": AuthMiddlewareStack(
        URLRouter([
            re_path(r'ws/map_route/$', MapRouteConsumer.as_asgi()),
            re_path(r'ws/update/data/$', EspConnectConsumer.as_asgi()),
        ])
    ),    
})