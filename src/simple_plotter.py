from rplidar.driver import RpLidarA1
import time
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import logging

logging.getLogger().setLevel(logging.INFO)


def update_plot(frame, *fargs):
    if not len(frame):
        return
    theta = np.array([pt['angle'] * 0.0174533 for pt in frame])
    r = np.array([pt['distance'] / 1000 for pt in frame])

    ax.cla()
    ax.scatter(theta, r, c='k', s=1)
    ax.set_rmax(7)

lidar = RpLidarA1('COM3')
lidar.get_device_info()
lidar.get_device_health()
lidar.get_sample_rate()
lidar.start_scan()
print(lidar.com.in_waiting)


fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_rlabel_position(-22.5)
ax.grid(True)

ani = FuncAnimation(fig, func=update_plot, frames=lidar.scan_data(), interval=1)
plt.show()

lidar.stop_scan()
print(lidar.com.in_waiting)
