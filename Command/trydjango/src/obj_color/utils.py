from asgiref.sync import async_to_sync
from channels.layers import get_channel_layer

from matplotlib import cm
from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
from PIL import Image

import io
import base64
import numpy as np

def plot_map(gmap_data):
    """
    helper function to plot two colormaps
    """
    top = cm.get_cmap('Greys', 128)
    bottom = cm.get_cmap('hsv', 128)

    newcolors = np.vstack((top(np.linspace(0, 1, 64)),
                        bottom(np.linspace(0, 1, 192))))
    newcmp = ListedColormap(newcolors, name='OrangeBlue')

    fig, axs = plt.subplots(figsize=(20,8))
    xydim = np.array(gmap_data).shape
    psm = axs.imshow(gmap_data, cmap=newcmp, rasterized=True, vmin=-4, vmax=4)
    psm.set_clim(0, 8)
    fig.colorbar(psm, ax=axs)
    plt.gca().set_xticks(np.arange(-.5, xydim[1], 1), minor = True)
    plt.gca().set_yticks(np.arange(-.5, xydim[0], 1), minor = True)
    plt.grid(True, which="minor", color="w", linewidth = .6, alpha = 0.5)

    flike = io.BytesIO()
    fig.savefig(flike, bbox_inches='tight',pad_inches = 0)
    # converts binary/bytes to base64 string
    b64 = base64.b64encode(flike.getvalue()).decode('utf-8')
    return b64

def update_websocket(map_fig):
    print("running update_websocket")
    flike = io.BytesIO()
    map_fig.savefig(flike, bbox_inches='tight',pad_inches = 0)
    # converts binary/bytes to base64 string
    b64_str = base64.b64encode(flike.getvalue()).decode('utf-8')

    channel_layer = get_channel_layer()
    async_to_sync(channel_layer.group_send)(
        "map",
        {
            "type": "map.message",
            "text": b64_str,
        },
    )
    flike.close()

def get_rover_angle(prev_angle, current_x, current_y, prev_x, prev_y):
    current_angle = -1
    if (current_x, current_y) != (prev_x, prev_y):
        if current_x > prev_x and current_y > prev_y:
                current_angle = 315
        elif current_x > prev_x and current_y < prev_y:
                current_angle = 45
        elif current_x < prev_x and current_y > prev_y:
            current_angle = 225
        elif current_x < prev_x and current_y < prev_y:
                current_angle = 135
        elif current_x == prev_x and current_y > prev_y:
            current_angle = 270
        elif current_x == prev_x and current_y < prev_y:
                current_angle = 90
        elif current_y == prev_y and current_x > prev_x:
            current_angle = 0
        elif current_y == prev_y and current_x < prev_x:
            current_angle = 180
    if current_angle == -1:
        print("New angle of rover not calculated")
    return current_angle

    