from django.urls import re_path

from espConnect import consumers

websocket_urlpatterns = [
    re_path(r'ws/update/data/$', consumers.EspConnectConsumer.as_asgi()),
]