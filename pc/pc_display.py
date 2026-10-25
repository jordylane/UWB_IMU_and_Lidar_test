import sys
import socket
import threading
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import UDP_PORT, decode_packet

PLOT_MAX_RANGE = 4.0
PLOT_AUTO_RANGE = False
PLOT_CONFIDENCE = True
PLOT_CONFIDENCE_COLOUR_MAP = "bwr_r"

anchors = np.array([
    [0.0, 0.0, 1.2],
    [0.0, 4.0, 0.75],
    [4.0, 4.0, 1.2],
    [4.0, 0.0, 0.75]
])


class DisplayData:
    def __init__(self):
        self.lock = threading.Lock()
        self.lidar_x = np.array([])
        self.lidar_y = np.array([])
        self.lidar_c = np.array([])
        self.uwb_x = 0.0
        self.uwb_y = 0.0
        self.distances = [0.0, 0.0, 0.0, 0.0]
        self.ax_buffer = deque(maxlen=100)
        self.ay_buffer = deque(maxlen=100)
        self.heading = 0.0
        self.yaw = 0.0
        self.timestamp = 0.0
        self.x_history = []
        self.y_history = []


def receiver_loop(shared_data, stop_event):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        udp_socket.bind(("0.0.0.0", UDP_PORT))
        udp_socket.settimeout(0.5)

        while not stop_event.is_set():
            try:
                payload, _ = udp_socket.recvfrom(65535)
            except socket.timeout:
                continue

            try:
                packet = decode_packet(payload)
                if not all(isinstance(packet[key], list) for key in (
                    "lidar_x", "lidar_y", "lidar_c", "distances"
                )):
                    continue
                lidar_x = np.asarray(packet["lidar_x"], dtype=float)
                lidar_y = np.asarray(packet["lidar_y"], dtype=float)
                lidar_c = np.asarray(packet["lidar_c"], dtype=float)
                distances = [float(value) for value in packet["distances"]]
                uwb_x = float(packet["uwb_x"])
                uwb_y = float(packet["uwb_y"])
                ax = float(packet["ax"])
                ay = float(packet["ay"])
                heading = float(packet["heading"])
                yaw = float(packet["yaw"])
                timestamp = float(packet["timestamp"])
                if (
                    len(distances) != 4
                    or lidar_x.ndim != 1
                    or lidar_y.ndim != 1
                    or lidar_c.ndim != 1
                    or not (len(lidar_x) == len(lidar_y) == len(lidar_c))
                ):
                    continue
            except (
                KeyError,
                TypeError,
                ValueError,
                UnicodeDecodeError,
                OverflowError,
                RecursionError,
            ):
                continue

            with shared_data.lock:
                shared_data.lidar_x = lidar_x
                shared_data.lidar_y = lidar_y
                shared_data.lidar_c = lidar_c
                shared_data.uwb_x = uwb_x
                shared_data.uwb_y = uwb_y
                shared_data.distances = distances
                shared_data.ax_buffer.append(ax)
                shared_data.ay_buffer.append(ay)
                shared_data.heading = heading
                shared_data.yaw = yaw
                shared_data.timestamp = timestamp
                shared_data.x_history.append(uwb_x)
                shared_data.y_history.append(uwb_y)


def display_loop(shared_data, stop_event):
    
    
    fig = plt.figure(figsize=(8, 8))
    
    ax_pos = fig.add_subplot(221)
    
    x_history = []
    y_history = []

    circle_1 = Circle(
        (anchors[0, 0], anchors[0, 1]),    # centre
        0,            # initial radius
        fill=False,
        color='red',
        linewidth=0.5
    )

    ax_pos.add_patch(circle_1)

    circle_2 = Circle(
        (anchors[1, 0], anchors[1, 1]),    # centre
        0,            # initial radius
        fill=False,
        color='red',
        linewidth=0.5
    )

    ax_pos.add_patch(circle_2)

    circle_3 = Circle(
        (anchors[2, 0], anchors[2, 1]),    # centre
        0,            # initial radius
        fill=False,
        color='red',
        linewidth=0.5
    )

    ax_pos.add_patch(circle_3)

    circle_4 = Circle(
        (anchors[3, 0], anchors[3, 1]),    # centre
        0,            # initial radius
        fill=False,
        color='red',
        linewidth=0.5
    )

    ax_pos.add_patch(circle_4)


    graph_history = ax_pos.plot([], [],'b-', label="Path")[0]
    graph_current = ax_pos.plot([], [],'ro', label="Current Position")[0]

    ax_pos.set_xlim(0, 4.0)
    ax_pos.set_ylim(0, 4.0)
    ax_pos.set_aspect('equal')

    ax_pos.set_title("UWB Position")
    ax_pos.set_xlabel("X (m)")
    ax_pos.set_ylabel("Y (m)")
    ax_pos.grid(True)

    
    ax_accel = fig.add_subplot(222)
    
    graph_accel_x = ax_accel.plot([], 'b-', label='x acceleration')[0]
    graph_accel_y = ax_accel.plot([], 'r-', label='y acceleration')[0]

    ax_accel.legend()
    ax_accel.set_title("Acceleration")
    ax_accel.grid(True)
    
    ax_compass = fig.add_subplot(223)

    t = np.linspace(
        0,
        2*np.pi,
        500
    )

    ax_compass.plot(
        np.cos(t),
        np.sin(t),
        'k',
        lw=2
    )

    for ang in range(0, 360, 10):

        a = np.radians(ang)

        r1 = 0.92
        r2 = 1.00

        ax_compass.plot(
            [r1*np.sin(a), r2*np.sin(a)],
            [r1*np.cos(a), r2*np.cos(a)],
            'k'
        )

    graph_compass_red = ax_compass.plot(
        [],
        [],
        'r',
        linewidth=5
    )[0]

    graph_compass_blue = ax_compass.plot(
        [],
        [],
        'b',
        linewidth=3
    )[0]

    ax_compass.text(
        0, 1.12, 'N',
        fontsize=16,
        ha='center'
    )

    ax_compass.text(
        1.12, 0, 'E',
        fontsize=16,
        ha='center'
    )

    ax_compass.text(
        0, -1.12, 'S',
        fontsize=16,
        ha='center'
    )

    ax_compass.text(
        -1.12, 0, 'W',
        fontsize=16,
        ha='center'
    )

    ax_compass.set_xlim([-1.2, 1.2])
    ax_compass.set_ylim([-1.2, 1.2])

    ax_compass.set_aspect('equal')
    ax_compass.axis('off')

    ax_compass.set_title(f"Compass Heading\n")

    
    ax_lidar = fig.add_subplot(224)
    
    # Initial lidar scatter plot
    if PLOT_CONFIDENCE:
        graph = ax_lidar.scatter([], [], c=[], marker=".", s=8, vmin=0,
                            vmax=255, cmap=PLOT_CONFIDENCE_COLOUR_MAP)
    else:
        graph = ax_lidar.plot([], [], ".")[0]

    
    # Limit to +/- PLOT_MAX_RANGE meters
    ax_lidar.set_xlim(-PLOT_MAX_RANGE,PLOT_MAX_RANGE)
    ax_lidar.set_ylim(-PLOT_MAX_RANGE,PLOT_MAX_RANGE)

    ax_lidar.set_xlim(-4, 4)
    ax_lidar.set_ylim(-4, 4)
    
    # Initial yaw arrow
    yaw_arrow = FancyArrowPatch(
        (0, 0),
        (0, 1),
        color='red',
        mutation_scale=15,
        linewidth=0.5
    )

    ax_lidar.add_patch(yaw_arrow)

    # Initial text
    yaw_text = ax_lidar.text(
        0,
        -PLOT_MAX_RANGE + 0.3,
        "Yaw = 0.0°",
        color='red'
    )
    
    plt.ion()
    plt.show(block=False)
    fig.canvas.mpl_connect("close_event", lambda event: stop_event.set())

    while not stop_event.is_set() and plt.fignum_exists(fig.number):
        
#         start = time.perf_counter()

        with shared_data.lock:

            x = shared_data.lidar_x
            y = shared_data.lidar_y
            c = shared_data.lidar_c
            distances = shared_data.distances.copy()

            heading = shared_data.heading
            yaw = shared_data.yaw

            ax_hist = np.asarray(shared_data.ax_buffer)
            ay_hist = np.asarray(shared_data.ay_buffer)
            x_history = list(shared_data.x_history)
            y_history = list(shared_data.y_history)


        a = np.radians(heading)

        hx = -np.sin(a)
        hy = np.cos(a)

        heading = (np.degrees(np.arctan2(hx, hy)) - 90 + 360) % 360

        yaw = (yaw - (np.pi / 2) + (2 * np.pi)) % (2 * np.pi)

        # =====================================
        # position Plot
        # =====================================
        
        circle_1.set_radius(distances[0])
        circle_2.set_radius(distances[1])
        circle_3.set_radius(distances[2])
        circle_4.set_radius(distances[3])
        
        graph_history.set_data(x_history, y_history)

        if x_history and y_history:
            graph_current.set_data([x_history[-1]], [y_history[-1]])
        else:
            graph_current.set_data([], [])

        graph_accel_x.set_data(
            np.arange(len(ax_hist)),
            ax_hist
        )

        graph_accel_y.set_data(
            np.arange(len(ay_hist)),
            ay_hist
        )

        if len(ax_hist):
            ax_accel.set_xlim(0, len(ax_hist))
            ymin = min(ax_hist.min(), ay_hist.min()) - 0.25
            ymax = max(ax_hist.max(), ay_hist.max()) + 0.25
            ax_accel.set_ylim(ymin, ymax)
        
        
        # =====================================
        # Compass Plot
        # =====================================

        a = np.radians(heading)

        graph_compass_red.set_data(
            [0, 0.8*np.sin(a)],
            [0, 0.8*np.cos(a)]
        )

        graph_compass_blue.set_data(
            [0, -0.3*np.sin(a)],
            [0, -0.3*np.cos(a)]
        )

        ax_compass.set_title(
            f"Compass Heading: {heading:.1f}°"
        )
        
        # =====================================
        # Lidar Plot
        # =====================================
        
        # Work out max coordinate, and set the scale based on this.
        # Force a 1:1 aspect ratio
        if PLOT_AUTO_RANGE and len(x) and len(y):
            mav_val = max(np.max(np.abs(x)), np.max(np.abs(y))) * 1.2
            ax_lidar.set_xlim(-mav_val, mav_val)
            ax_lidar.set_ylim(-mav_val, mav_val)
            
        
        # Plot the new data
        if PLOT_CONFIDENCE:
            points = np.column_stack((x, y))
            graph.set_offsets(points)
            graph.set_array(np.asarray(c))
        else:
            graph = ax_lidar.plot(x,y,'b.')[0]
            
        # ---------------------------------------
        # Plot yaw direction arrow
        # ---------------------------------------


        # Arrow length in metres
        arrow_length = 1.0

        # Convert yaw to XY vector
        arrow_x = arrow_length * np.sin(yaw)
        arrow_y = arrow_length * np.cos(yaw)

        # Update existing arrow
        yaw_arrow.set_positions(
        (0, 0),
        (arrow_x, arrow_y)
        )

        # Update text
        yaw_text.set_text(
            f"Yaw = {np.degrees(yaw):.1f}°"
        )
            
        plt.pause(0.03)


def main():
    shared_data = DisplayData()
    stop_event = threading.Event()
    receiver = threading.Thread(
        target=receiver_loop,
        args=(shared_data, stop_event),
        daemon=True,
    )
    receiver.start()

    try:
        display_loop(shared_data, stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        plt.close("all")


if __name__ == "__main__":
    main()
