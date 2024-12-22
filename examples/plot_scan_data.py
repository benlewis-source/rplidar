import matplotlib
import numpy as np

from rplidar.rplidar_driver import *

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def render_frame(frame, *fargs):
    if not len(frame):
        return

    theta = np.array([np.deg2rad(pt.angle) for pt in frame if pt.quality])
    r = np.array([pt.distance / 1000 for pt in frame if pt.quality])

    ax.cla()
    ax.scatter(theta, r, c="k", s=1)
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rlabel_position(45)
    ax.set_rmax(2.5)


if __name__ == "__main__":
    lidar = RPLidarA1M8("COM3")
    lidar.reset()
    lidar.startMotor()

    try:
        func = lidar.startScanExpress(MODE_EXPRESS)

        fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
        ax.grid(True)
        ani = FuncAnimation(fig, func=render_frame, frames=lidar.getScanDataFrames(func), interval=0.02)
        plt.show()

    finally:
        lidar.stop()
        lidar.stopMotor()
