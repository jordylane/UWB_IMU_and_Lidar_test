import serial
import threading
import time
import struct
from enum import Enum
from collections import deque
import socket
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from pynput import keyboard

from common import PI_IP, CONTROL_RATE_HZ

USE_LIVE_DATA = False

if USE_LIVE_DATA:
    from sense_hat import SenseHat

# --------------------------------------------------
# Data Sources
# --------------------------------------------------

class SerialLidarSource:

    def __init__(self, port, record_filename=None):

        self.source = serial.Serial(
            port,
            230400,
            timeout=0.5
        )

        self.record_file = None

        if record_filename:
            self.record_file = open(record_filename, "wb")

    def read(self, size=1):

        data = self.source.read(size)

        if self.record_file and data:
            self.record_file.write(data)

        return data

    def close(self):

        if self.record_file:
            self.record_file.close()

        self.source.close()


class FileLidarSource:

    def __init__(self, filename, baudrate=230400):
        self.source = open(filename, "rb")
        self.baudrate = baudrate

        # Playback timing
        self.next_read_time = time.perf_counter()

    def read(self, size=1):

        data = self.source.read(size)

        if len(data) < size:
            self.source.seek(0)
            data += self.source.read(size - len(data))

            with shared.lock:
                shared.lidar_file_looped = True

        # Serial transfer time for this block
        transfer_time = len(data) * 12.925 / self.baudrate

        # Schedule next transfer completion time
        self.next_read_time += transfer_time

        # Wait until that time arrives
        while True:

            remaining = self.next_read_time - time.perf_counter()

            if remaining <= 0:
                break

            # Sleep when far away from target
            if remaining > 0.001:
                time.sleep(remaining - 0.0005)

        return data

    def close(self):
        self.source.close()


class SerialUWBSource:

    def __init__(self, port, baudrate=115200,
                 record_filename=None):

        self.source = serial.Serial(
            port,
            baudrate,
            timeout=1
        )

        self.record_file = None

        if record_filename:
            self.record_file = open(
                record_filename,
                "w"
            )

    def readline(self):

        data = self.source.readline()
            
        if self.record_file and data:

            line = data.decode("utf-8")

            self.record_file.write(line)
            self.record_file.flush()

        return data

    def close(self):

        if self.record_file:
            self.record_file.close()

        self.source.close()
        

class FileUWBSource:

    def __init__(self,
                 filename,
                 baudrate=115200):

        self.filename = filename
        self.source = open(filename, "r")
        self.baudrate = baudrate

        # Playback timing
        self.next_read_time = time.perf_counter()

    def readline(self):

        line = self.source.readline()

        if line == "":
            self.source.seek(0)
            line = self.source.readline()
            with shared.lock:
                shared.uwb_file_looped = True

        # Serial transfer time for this block
        transfer_time = len(line) * 10.0 / self.baudrate

        # Schedule next transfer completion time
        self.next_read_time += transfer_time

        # Wait until that time arrives
        while True:

            remaining = self.next_read_time - time.perf_counter()

            if remaining <= 0:
                break

            # Sleep when far away from target
            if remaining > 0.001:
                time.sleep(remaining - 0.0005)

        return line.encode("utf-8")

    def close(self):

        self.source.close()


class LiveAccSource:

    def __init__(self, record_filename=None):

        self.sense = SenseHat()

        self.record_file = None

        if record_filename:
            self.record_file = open(record_filename, "w")

    def get_data(self, elapsed = 0.0):

        accel = self.sense.get_accelerometer_raw()
        
        accel_x = accel['x']
        accel_y = accel['y']
        accel_z = accel['z']
        
        if self.record_file:
            self.record_file.write(
                f"{accel_x:.4f},{accel_y:.4f},{accel_z:.4f},{elapsed}\n"
            )
            self.record_file.flush()         
    
        return accel_x, accel_y, accel_z, elapsed
    
    def close(self):

        if self.record_file:
            self.record_file.close()
            
class FileAccSource:

    def __init__(self,
                 filename,
                 sample_period=0.01):

        self.file = open(filename, "r")
        self.sample_period = sample_period

    def get_data(self, elapsed = 0.0):

        line = self.file.readline()

        if line == "":
            self.file.seek(0)
            line = self.file.readline()
            with shared.lock:
                shared.acc_file_looped = True

        accel_x, accel_y, accel_z, elapsed = map(
            float,
            line.strip().split(",")
        )

        time.sleep(self.sample_period)

        return accel_x, accel_y, accel_z, elapsed

    def close(self):

        self.file.close()


class LiveAttSource:

    def __init__(self, record_filename=None):

        self.sense = SenseHat()

        self.record_file = None

        if record_filename:
            self.record_file = open(record_filename, "w")

    def get_data(self, elapsed=0.0):

        attitude = self.sense.get_orientation_radians()

        roll = attitude["roll"]
        pitch = attitude["pitch"]
        yaw = attitude["yaw"]

        mag = self.sense.get_compass_raw()

        mx = mag["x"]
        my = mag["y"]

        heading = np.degrees(np.arctan2(my, mx))

        if heading < 0:
            heading += 360

        if self.record_file:
            self.record_file.write(
                f"{heading:.4f},{roll:.4f},{pitch:.4f},{yaw:.4f},{elapsed}\n"
            )
            self.record_file.flush()

        return heading, roll, pitch, yaw, elapsed
    

    def close(self):

        if self.record_file:
            self.record_file.close()
            
            
class FileAttSource:

    def __init__(self,
                 filename,
                 sample_period=0.05):

        self.file = open(filename, "r")
        self.sample_period = sample_period

    def get_data(self, elapsed = 0.0):

        line = self.file.readline()

        if line == "":
            self.file.seek(0)
            line = self.file.readline()
            with shared.lock:
                shared.att_file_looped = True

        heading, roll, pitch, yaw, elapsed = map(
            float,
            line.strip().split(",")
        )

        time.sleep(self.sample_period)

        return heading, roll, pitch, yaw, elapsed

    def close(self):

        self.file.close()
            
# --------------------------------------------------
# Shared data
# --------------------------------------------------

class SharedData:
    def __init__(self):
        self.lock = threading.Lock()
        
        self.ax_buffer = deque(maxlen=100)
        self.ay_buffer = deque(maxlen=100)

        self.uwb_x = 0.0
        self.uwb_y = 0.0
        
        self.x = 0.0
        self.y = 0.0
        self.c = 0.0
        
        self.heading = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.uwb_elapsed_time = 0.0
        self.lidar_elapsed_time = 0.0
        self.acc_elapsed_time = 0.0
        self.att_elapsed_time = 0.0
        
        self.uwb_file_looped = False
        self.acc_file_looped = False
        self.att_file_looped = False
        self.lidar_file_looped = False

        
def trilaterate_3d(anchors, distances):
    """
    3D trilateration using four anchors.

    Parameters
    ----------
    anchors : ndarray (4,3)
        Anchor coordinates [[x1,y1,z1], ... [x4,y4,z4]]

    distances : ndarray (4,)
        Measured distances [d1,d2,d3,d4]

    Returns
    -------
    ndarray (3,)
        Estimated position [x,y,z]
    """

    anchors = np.asarray(anchors, dtype=float)
    distances = np.asarray(distances, dtype=float)

    x1, y1, z1 = anchors[0]
    d1 = distances[0]

    A = []
    b = []

    for i in range(1, 4):
        xi, yi, zi = anchors[i]
        di = distances[i]

        A.append([
            2 * (xi - x1),
            2 * (yi - y1),
            2 * (zi - z1)
        ])

        b.append(
            d1**2 - di**2
            + xi**2 - x1**2
            + yi**2 - y1**2
            + zi**2 - z1**2
        )

    A = np.array(A)
    b = np.array(b)

    position = np.linalg.solve(A, b)

    return position


# Example
anchors = np.array([
    [0.0, 0.0, 1.2],   # Anchor 1
    [0.0, 4.0, 0.75],   # Anchor 2
    [4.0, 4.0, 1.2],   # Anchor 3
    [4.0, 0.0, 0.75]    # Anchor 4
])

distances = [0, 0, 0, 0]

# ----------------------------------------------------------------------
# Lidar System Constants
# ----------------------------------------------------------------------
# Serial port of the LIDAR unit
SERIAL_PORT = "/dev/ttyAMA0"

# At the default rotation speed (~3600 deg/s) the system outputs about 
# 480 measurements in a full rotation. We want to plot at least this
# many in order to get a 360 degree plot
MEASUREMENTS_PER_PLOT = 480

# System will plot between +/-PLOT_MAX_RANGE on both X and Y axis
PLOT_MAX_RANGE = 4.0 # in meters

# Set the plot area to autoscale to the max distance in X or Y
PLOT_AUTO_RANGE = False

# Show the confidence as a colour gradiant on the plot
PLOT_CONFIDENCE = True

# Set the colour gradiant to use. See this URL for options:
# https://matplotlib.org/stable/gallery/color/colormap_reference.html
PLOT_CONFIDENCE_COLOUR_MAP = "bwr_r"

#Enable debug messages
PRINT_DEBUG = False

# ----------------------------------------------------------------------
# Main Packet Format
# ----------------------------------------------------------------------

# All fields are little endian
# Header (1 byte) = 0x54
# Length (1 byte) = 0x2C (assumed to be constant)
# Speed (2 bytes) Rotation speed in degrees per second
# Start angle (2 bytes) divide by 100.0 to get angle in degrees
# Data Measurements (MEASUREMENT_LENGTH * 3 bytes)
#                   See "Format of each data measurement" below
# Stop angle (2 bytes) divide by 100.0 to get angle in degrees
# Timestamp (2 bytes) In milliseconds
# CRC (1 bytes) Poly: 0x4D, Initial Value: 0x00, Final Xor Value: 0x00
#               Input reflected: False, Result Reflected: False
#               http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
# Format of each data measurement
# Distance (2 bytes) # In millimeters
# Confidence (1 byte)

# ----------------------------------------------------------------------
# Packet format constants
# ----------------------------------------------------------------------

PACKET_LENGTH = 47
MEASUREMENT_LENGTH = 12 
MESSAGE_FORMAT = "<xBHH" + "HB" * MEASUREMENT_LENGTH + "HHB"

State = Enum("State", ["SYNC0", "SYNC1", "SYNC2", "LOCKED", "UPDATE_PLOT"])


def parse_lidar_data(data):
    
    # Extract data
    length, speed, start_angle, *pos_data, stop_angle, timestamp, crc = \
        struct.unpack(MESSAGE_FORMAT, data)
    
    # Scale values
    start_angle = float(start_angle) / 100.0
    stop_angle = float(stop_angle) / 100.0
    
    # Unwrap angle if needed and calculate angle step size
    if stop_angle < start_angle:
        stop_angle += 360.0
    step_size = (stop_angle - start_angle) / (MEASUREMENT_LENGTH - 1)
    
    # Get the angle for each measurement in packet
    angle = [start_angle + step_size * i for i in range(0,MEASUREMENT_LENGTH)]
    distance = pos_data[0::2] # in millimeters
    confidence = pos_data[1::2]
    N = len(angle)
    time_arr = np.full(12, timestamp)

    if PRINT_DEBUG:
        print(length, speed, start_angle, *pos_data, stop_angle, timestamp, crc)
        
    return list(zip(angle, distance, confidence, time_arr))

def get_xyc_data(measurements):
    # Unpack the tuples
    angle = np.array([measurement[0] for measurement in measurements])
    distance = np.array([measurement[1] for measurement in measurements])
    confidence = np.array([measurement[2] for measurement in measurements])
    time = np.array([measurement[3] for measurement in measurements])
    # Convert to cartesian coordinates in meters
    x = np.sin(np.radians(angle)) * (distance / 1000.0)
    y = np.cos(np.radians(angle)) * (distance / 1000.0)
    return x, y, confidence, time[0]


shared = SharedData()
running = True

# --------------------------------------------------
# Lidar Thread
# --------------------------------------------------

def lidar_loop():
    
# --------------------------------------------------
# Choose source
# --------------------------------------------------

    if USE_LIVE_DATA:
        source = SerialLidarSource(SERIAL_PORT, record_filename="lidar_capture.bin")
    else:
        source = FileLidarSource("lidar_capture.bin")
    

    new_sample = False
    sample_time = 0.0
    prev_sample_time = 0.0
    num_samples = 0
    elapsed_sample_time = 0
    elapsed = 0
    start_time = time.perf_counter()
    
    # Set up initial state
    measurements = []
    data = b''
    state = State.SYNC0

    # Main state machine
    while running:
        # Find 1st header byte
        if state == State.SYNC0:
            
            data = b''
            measurements = []
            
            if source.read() == b'\x54':
                data = b'\x54'
                state = State.SYNC1
                
        # Find 2nd header byte
        # Technically this is the length field but the packet length
        # is fixed, so it can be treated as a constant.
        elif state == State.SYNC1:
            
            if source.read() == b'\x2C':
                state = State.SYNC2
                data += b'\x2C'
            else:
                state = State.SYNC0
                
        # Read remainder of the packet (PACKET_LENGTH minus the 2 header
        # bytes we have already read).
        elif state == State.SYNC2:
            
            data += source.read(size=PACKET_LENGTH - 2)
            
            if len(data) != PACKET_LENGTH:
                state = State.SYNC0
                continue
            
            measurements += parse_lidar_data(data)
            
            state = State.LOCKED
            
        elif state == State.LOCKED:
            
            data = source.read(size=PACKET_LENGTH)
            
            if data[0] != 0x54 or len(data) != PACKET_LENGTH:
                print("WARNING: Serial sync lost")
                state = State.SYNC0
                continue
            
            measurements += parse_lidar_data(data)
            
            if len(measurements) > MEASUREMENTS_PER_PLOT:
                state = State.UPDATE_PLOT
                
        elif state == State.UPDATE_PLOT:
            
            x, y, c, sample_time = get_xyc_data(measurements)
            
            with shared.lock:
                shared.x = x
                shared.y = y
                shared.c = c
                
            # Get the next packet
            state = State.LOCKED
            measurements = []
            
            
        if sample_time != prev_sample_time:
            new_sample = True
            num_samples += 1
            prev_sample_time = sample_time
        else:
            new_sample = False
            
        if num_samples == 1:
            sample_start_time = sample_time
            
        elapsed += time.perf_counter() - start_time
        start_time = time.perf_counter()
            
        if new_sample:
            
            if shared.lidar_file_looped:
                shared.lidar_file_looped = False
                sample_start_time = sample_time - sample_duration
            
            sample_duration = sample_time - sample_start_time 
                
            if sample_duration < 0:
                elapsed_sample_time += sample_duration + 30000
            else:
                elapsed_sample_time += sample_duration
                
            sample_start_time = sample_time
                
        with shared.lock:
            shared.lidar_elapsed_time = elapsed_sample_time/1000
            
#         if num_samples % 10 == 0.0 and new_sample:
#             print(f"lidar elapsed = {elapsed_sample_time/1000}")
            
            
# --------------------------------------------------
# UWB Thread
# --------------------------------------------------

def uwb_loop():
        
    if USE_LIVE_DATA:
        source = SerialUWBSource('/dev/ttyUSB0', record_filename="uwb_capture.txt")
    else:
        source = FileUWBSource("uwb_capture.txt", baudrate=115200)
    
    
    new_sample = False
    sample_time = 0.0
    prev_sample_time = 0.0
    num_samples = 0
    uwb_elapsed_millis = 0
    uwb_elapsed = 0.0
    elapsed = 0.0
    start_time = time.perf_counter()
    time_delay = 0.0
    
    time_step = 1.0
    
    Kp = 0.1
    Ki = 0.01
    integral = 0.0
    
    while running:
    
        # Read UWB position
        line = source.readline().decode().strip()
        
        if line.startswith("time: "):
            try:
                sample_time = float(line.split(":")[1].strip())
            except ValueError:
                pass
        
        if line.startswith("1> "):
            try:
                distances[0], _ = map(float, line.replace("1> ","").split("\t"))
            except ValueError:
                pass
            
        if line.startswith("2> "):
            try:
                distances[1], _ = map(float, line.replace("2> ","").split("\t"))
            except ValueError:
                pass
        if line.startswith("3> "):
            try:
                distances[2], _ = map(float, line.replace("3> ","").split("\t"))
            except ValueError:
                pass
        if line.startswith("4> "):
            try:
                distances[3], _ = map(float, line.replace("4> ","").split("\t"))
            except ValueError:
                pass
            
        estimated_pos = trilaterate_3d(anchors, distances)
        
        with shared.lock:
            shared.uwb_x = estimated_pos[0]
            shared.uwb_y = estimated_pos[1]
        
        if sample_time != prev_sample_time:
            new_sample = True
            num_samples += 1
            prev_sample_time = sample_time
        else:
            new_sample = False
            
        if num_samples == 1:
            sample_start_time = sample_time
            
            
        if new_sample:
            
            if shared.uwb_file_looped:
                shared.uwb_file_looped = False
                sample_start_time = sample_time - sample_duration
            
            sample_duration = sample_time - sample_start_time
                
            uwb_elapsed_millis += sample_duration
            uwb_elapsed = uwb_elapsed_millis/1000
            sample_start_time = sample_time
                
        with shared.lock:
            lidar_elapsed = shared.lidar_elapsed_time
            shared.uwb_elapsed_time = uwb_elapsed
        
        now = time.perf_counter()
        dt = now - start_time
        elapsed += dt
        start_time = now
        
        error = lidar_elapsed - uwb_elapsed
        
        integral += error * dt
        correction = Kp*error + Ki*integral
        time_delay -= correction
        
        if time_delay < 0.0:
            time_delay = 0.0
            
        # if uwb_elapsed > time_step:
        #     time_step += 1.0
        #     print(f"elapsed = {elapsed}")
        #     print(f"lidar elapsed = {lidar_elapsed}")
        #     print(f"uwb elapsed = {uwb_elapsed}")
        #     print(f"uwb time delay = {time_delay}\n")
            
        time.sleep(time_delay)


# --------------------------------------------------
# Accelerometer Thread
# --------------------------------------------------

def accelerometer_loop():
    
# --------------------------------------------------
# Choose source
# --------------------------------------------------

    if USE_LIVE_DATA:
        acc_source = LiveAccSource(record_filename="accelerometer_capture.csv")
    else:
        acc_source = FileAccSource("accelerometer_capture.csv", sample_period=0.001)


    elapsed = 0.0
    prev_acc_elapsed = 0.0
    acc_elapsed_loop_offset = 0.0
    start_time = time.perf_counter()
    time_delay = 0.0
    
    time_step = 1.0
    
    Kp = 0.1
    Ki = 0.01
    integral = 0.0

    while running:

        accel_x, accel_y, accel_z, acc_elapsed = acc_source.get_data(elapsed)

        with shared.lock:
            shared.ax_buffer.append(accel_x)
            shared.ay_buffer.append(accel_y)
            lidar_elapsed = shared.lidar_elapsed_time
            shared.acc_elapsed_time = acc_elapsed
            
            if shared.acc_file_looped:
                acc_elapsed_loop_offset = prev_acc_elapsed
                shared.acc_file_looped = False

        acc_elapsed += acc_elapsed_loop_offset
        prev_acc_elapsed = acc_elapsed
        
        now = time.perf_counter()
        dt = now - start_time
        elapsed += dt
        start_time = now
        
        error = lidar_elapsed - acc_elapsed
        
        integral += error * dt
        correction = Kp*error + Ki*integral
        time_delay -= correction
        
        if time_delay < 0:
            time_delay = 0
        
#         if acc_elapsed > time_step:
#             time_step += 1.0
#             print(f"lidar elapsed = {lidar_elapsed}")
#             print(f"acc elapsed = {acc_elapsed}")
#             print(f"acc time delay = {time_delay}\n")
        
        if USE_LIVE_DATA:
            time.sleep(0.1)
        else:
            time.sleep(time_delay)
            
        


# --------------------------------------------------
# Attitude Thread
# --------------------------------------------------

def attitude_loop():
    
    if USE_LIVE_DATA:
        att_source = LiveAttSource(record_filename="attitude_capture.csv")
    else:
        att_source = FileAttSource("attitude_capture.csv", sample_period=0.0365)

    elapsed = 0.0
    prev_att_elapsed = 0.0
    att_elapsed_loop_offset = 0.0
    start_time = time.perf_counter()
    time_delay = 0.2
    
    time_step = 1.0
    
    Kp = 0.1
    Ki = 0.01
    integral = 0.0

    while running:

        heading, roll, pitch, yaw, att_elapsed = att_source.get_data(elapsed)

        with shared.lock:
            shared.heading = heading
            shared.roll = roll
            shared.pitch = pitch
            shared.yaw = yaw
            lidar_elapsed = shared.lidar_elapsed_time
            shared.att_elapsed_time = att_elapsed

            if shared.att_file_looped:
                att_elapsed_loop_offset = prev_att_elapsed
                shared.att_file_looped = False
                
        att_elapsed += att_elapsed_loop_offset
        prev_att_elapsed = att_elapsed
        
        now = time.perf_counter()
        dt = now - start_time
        elapsed += dt
        start_time = now
        
        error = lidar_elapsed - att_elapsed
        
        integral += error * dt
        correction = Kp*error + Ki*integral
        time_delay -= correction
        
        if time_delay < 0.05:
            time_delay = 0.05
            
#         if att_elapsed > time_step:
#             time_step += 1.0
#             print(f"lidar elapsed = {lidar_elapsed}")
#             print(f"att elapsed = {att_elapsed}")
#             print(f"att time delay = {time_delay}\n")
        
        
        if USE_LIVE_DATA:
            time.sleep(0.1)
        else:
            time.sleep(time_delay)

# --------------------------------------------------
# Display Thread
# --------------------------------------------------

def display_loop():
    
    
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
    
    from matplotlib.patches import FancyArrowPatch

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
    plt.show()

    while running:
        
#         start = time.perf_counter()

        with shared.lock:

            x = shared.x
            y = shared.y
            c = shared.c

            heading = shared.heading
            yaw = shared.yaw

            ax_hist = np.asarray(shared.ax_buffer)
            ay_hist = np.asarray(shared.ay_buffer)
            
            x_history.append(shared.uwb_x)
            y_history.append(shared.uwb_y)


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

        graph_current.set_data(
            [x_history[-1]],
            [y_history[-1]]
        )

        graph_accel_x.set_data(
            np.arange(len(ax_hist)),
            ax_hist
        )

        graph_accel_y.set_data(
            np.arange(len(ay_hist)),
            ay_hist
        )

        ax_accel.set_xlim(0, len(ax_hist))

        if len(ax_hist):
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
        if PLOT_AUTO_RANGE:
            mav_val = max([max(abs(x)), max(abs(y))]) * 1.2
            ax_lidar.xlim(-mav_val,mav_val)
            ax_lidar.ylim(-mav_val,mav_val)
            
        
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
            
        fig.canvas.flush_events()


# --------------------------------------------------
# ControlThread
# --------------------------------------------------

def control_loop():
    
    held = set()
    enabled = False
    lock = threading.Lock()
    keys = {keyboard.Key.left: b"L", keyboard.Key.right: b"R",
            keyboard.Key.up: b"U", keyboard.Key.down: b"D"}
    def on_press(key):
        nonlocal enabled
        if key == keyboard.Key.space:
            with lock:
                enabled = not enabled
                held.clear()
                state = enabled
            print("control ENABLED" if state else "control DISABLED")
        elif key in keys:
            with lock:
                held.add(key)
    def on_release(key):
        if key in keys:
            with lock:
                held.discard(key)
    listener = keyboard.Listener(on_press=on_press, on_release=on_release, daemon=True)
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_command = ""
    print("control_loop ready: press SPACE to enable/disable")
    try:
        listener.start()
        while running:
            with lock:
                commands = [value for key, value in keys.items() if enabled and key in held]
            for command in commands:
                try:
                    udp_socket.sendto(command, (PI_IP, CONTROL_PORT))
                except OSError:
                    pass
            command_string = "+".join(command.decode() for command in commands)
            if command_string != last_command:
                print(command_string if command_string else "released")
                last_command = command_string
            time.sleep(1 / CONTROL_RATE_HZ)
    finally:
        try:
            listener.stop()
        finally:
            udp_socket.close()

# --------------------------------------------------
# Start Threads
# --------------------------------------------------

acc_thread = threading.Thread(target=accelerometer_loop)
att_thread = threading.Thread(target=attitude_loop)
uwb_thread = threading.Thread(target=uwb_loop)
display_thread = threading.Thread(target=display_loop)
lidar_thread = threading.Thread(target=lidar_loop)
control_thread = threading.Thread(target=control_loop)


lidar_thread.start()
uwb_thread.start()
acc_thread.start()
att_thread.start()
display_thread.start()
control_thread.start()


try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:

    running = False

    acc_thread.join()
    att_thread.join()
    uwb_thread.join()
    lidar_thread.join()
    display_thread.join()

    
    print("Stopped")