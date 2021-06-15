from django.urls import re_path

from map_route import consumers

websocket_urlpatterns = [
    re_path(r'ws/map_route/$', consumers.MapRouteConsumer.as_asgi()),
]