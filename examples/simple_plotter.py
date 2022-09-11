from rplidar.interface import RpLidarA1M8
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import logging

logging.getLogger().setLevel(logging.DEBUG)
logging.getLogger('matplotlib').setLevel(logging.INFO)


def update_plot(frame, *fargs):
    if not len(frame):
        return
    theta = np.array([pt['radian'] for pt in frame if pt['quality']])
    r = np.array([pt['distance'] / 1000 for pt in frame if pt['quality']])

    ax.cla()
    ax.scatter(theta, r, c='k', s=1)
    # ax.plot(theta, r, c='k')
    ax.set_rmax(3)

lidar = RpLidarA1M8('COM3')
print(lidar.get_device_info())
print(lidar.get_device_health())
print(lidar.get_sample_rate())

try:
    lidar.start_scan()

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    ax.set_rlabel_position(-22.5)
    ax.grid(True)

    ani = FuncAnimation(fig, func=update_plot, frames=lidar.scan_data(), interval=1)
    plt.show()

finally:
    lidar.stop()
