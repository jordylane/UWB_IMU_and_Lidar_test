import math
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QPoint, Qt, QSize, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common import (
    CONTROL_PORT,
    CONTROL_RATE_HZ,
    PI_IP,
    UDP_PORT,
    anchors,
    decode_packet,
    pack_control_packet,
)

PRINT_EVERY = 10
PLOT_SPAN = 5.0
HISTORY_LENGTH = 100
CONTROL_KEYS = {
    b"L": "left",
    b"R": "right",
    b"U": "up",
    b"D": "down",
    b"w": "grabber_up",
    b"s": "grabber_down",
    b"d": "claw_open",
    b"a": "claw_close",
}

WINDOW = "#1e1f22"
PANEL = "#2b2d31"
BORDER = "#3a3c41"
TEXT = "#dbdee1"
MUTED = "#949ba4"
BLUE = "#4f9cf9"
GREEN = "#3ba55d"
RED = "#ed4245"
AMBER = "#f0b232"


def display_heading(angle):
    hx = -np.sin(angle)
    hy = np.cos(angle)
    return (np.degrees(np.arctan2(hx, hy)) - 90 + 360) % 360


def filter_position(x, y):
    return x, y


class DisplayData:
    def __init__(self):
        self.lock = threading.Lock()
        self.lidar_x = np.array([])
        self.lidar_y = np.array([])
        self.lidar_c = np.array([])
        self.ax_buffer = deque(maxlen=100)
        self.ay_buffer = deque(maxlen=100)
        self.uwb_x = 0.0
        self.uwb_y = 0.0
        self.distances = [0.0] * 4
        self.heading = 0.0
        self.yaw = 0.0
        self.seq = None
        self.rssi_dbm = None
        self.packet_count = 0
        self.last_packet_time = None
        self.sender_address = None
        self.arrival_times = deque(maxlen=HISTORY_LENGTH)
        self.sequence_history = deque(maxlen=HISTORY_LENGTH)


def receiver_loop(shared_data, stop_event, udp_socket):
    packet_count = 0
    failures = 0
    last_packet_time = time.monotonic()
    warned = False
    print(f"listening on port {UDP_PORT}", flush=True)

    while not stop_event.is_set():
        try:
            payload, sender_address = udp_socket.recvfrom(65535)
        except socket.timeout:
            if not warned and time.monotonic() - last_packet_time >= 5:
                print("no packets for 5 s", flush=True)
                warned = True
            continue
        except OSError as exc:
            if not stop_event.is_set():
                print(f"UDP receive failed: {exc}", flush=True)
            break

        received_at = time.monotonic()
        last_packet_time = received_at
        warned = False
        packet_count += 1

        try:
            packet = decode_packet(payload)
            lidar_x = np.asarray(packet["lidar_x"])
            lidar_y = np.asarray(packet["lidar_y"])
            lidar_c = np.asarray(packet["lidar_c"])
            distances = list(packet["distances"])
            seq = packet.get("seq")
            rssi_dbm = packet.get("rssi_dbm")
            with shared_data.lock:
                shared_data.lidar_x = lidar_x
                shared_data.lidar_y = lidar_y
                shared_data.lidar_c = lidar_c
                shared_data.ax_buffer.append(float(packet["ax"]))
                shared_data.ay_buffer.append(float(packet["ay"]))
                shared_data.uwb_x = float(packet["uwb_x"])
                shared_data.uwb_y = float(packet["uwb_y"])
                shared_data.distances = distances
                shared_data.heading = float(packet["heading"])
                shared_data.yaw = float(packet["yaw"])
                shared_data.seq = seq if isinstance(seq, int) else None
                shared_data.rssi_dbm = (
                    float(rssi_dbm)
                    if isinstance(rssi_dbm, (int, float))
                    and math.isfinite(rssi_dbm)
                    else None
                )
                shared_data.packet_count = packet_count
                shared_data.last_packet_time = received_at
                shared_data.sender_address = sender_address
                shared_data.arrival_times.append(received_at)
                if shared_data.seq is not None:
                    shared_data.sequence_history.append(shared_data.seq)

            if packet_count % PRINT_EVERY:
                continue
            distances_text = ", ".join(
                f"{value:.2f}" for value in packet["distances"]
            )
            age = time.time() - packet["timestamp"]
            print(
                f"#{packet_count} {len(payload)} B "
                f"lidar={len(packet['lidar_x'])} "
                f"uwb=({packet['uwb_x']:.2f}, {packet['uwb_y']:.2f}) "
                f"distances=[{distances_text}] "
                f"heading={packet['heading']:.1f} "
                f"yaw={packet['yaw']:.1f} age={age:.3f}s",
                flush=True,
            )
        except (KeyError, TypeError, ValueError, RecursionError) as exc:
            failures += 1
            if failures % 50 == 0:
                print(f"decode failure {failures}: {exc}", flush=True)


class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.angle = 0.0
        self.setFixedSize(150, 150)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

    def set_heading(self, angle):
        self.angle = angle
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        center = self.rect().center()
        radius = min(self.width(), self.height()) * 0.43

        painter.setPen(QPen(QColor(BORDER), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(center, int(radius), int(radius))

        for degrees in range(0, 360, 10):
            angle = math.radians(degrees)
            inner = radius * (0.88 if degrees % 30 == 0 else 0.93)
            outer_x = center.x() + math.sin(angle) * radius
            outer_y = center.y() - math.cos(angle) * radius
            inner_x = center.x() + math.sin(angle) * inner
            inner_y = center.y() - math.cos(angle) * inner
            painter.setPen(QPen(QColor(MUTED), 1))
            painter.drawLine(
                int(inner_x), int(inner_y), int(outer_x), int(outer_y)
            )

        painter.setPen(QColor(TEXT))
        for label, degrees in (("N", 0), ("E", 90), ("S", 180), ("W", 270)):
            angle = math.radians(degrees)
            x = center.x() + math.sin(angle) * radius * 0.70
            y = center.y() - math.cos(angle) * radius * 0.70
            painter.drawText(
                int(x - 6), int(y + 5), label
            )

        angle = math.radians(self.angle)
        painter.setPen(QPen(QColor(RED), 3))
        painter.drawLine(
            center,
            center + QPoint(
                int(math.sin(angle) * radius * 0.65),
                int(-math.cos(angle) * radius * 0.65),
            ),
        )
        painter.setPen(QPen(QColor(BLUE), 3))
        painter.drawLine(
            center,
            center + QPoint(
                int(-math.sin(angle) * radius * 0.28),
                int(math.cos(angle) * radius * 0.28),
            ),
        )


class SquarePlotWidget(pg.PlotWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        policy = QSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Preferred,
        )
        policy.setHeightForWidth(True)
        self.setSizePolicy(policy)

    def hasHeightForWidth(self):
        return True

    def heightForWidth(self, width):
        return width


class LidarPlot(SquarePlotWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setBackground(PANEL)
        self.showGrid(x=True, y=True, alpha=0.25)
        self.setLabel("bottom", "X", units="m")
        self.setLabel("left", "Y", units="m")
        self.setMouseEnabled(False, False)
        self.setXRange(-PLOT_SPAN / 2, PLOT_SPAN / 2, padding=0)
        self.setYRange(-PLOT_SPAN / 2, PLOT_SPAN / 2, padding=0)
        self.setAspectLocked(True, ratio=1)
        self.disableAutoRange()

        self.lidar_points = pg.ScatterPlotItem(size=3, pen=None)
        self.plotItem.addItem(self.lidar_points)
        self.heading_line = self.plot([], [], pen=pg.mkPen(RED, width=2))
        self.heading_head_left = self.plot([], [], pen=pg.mkPen(RED, width=2))
        self.heading_head_right = self.plot([], [], pen=pg.mkPen(RED, width=2))
        self.yaw_text = pg.TextItem(
            "Yaw = 0.0°", color=RED, anchor=(0, 1)
        )
        self.plotItem.addItem(self.yaw_text)
        self.yaw_text.setPos(-PLOT_SPAN / 2 + 0.1, -PLOT_SPAN / 2 + 0.1)

    def update_data(self, lidar_x, lidar_y, lidar_c, yaw):
        lidar_x = np.asarray(lidar_x, dtype=float)
        lidar_y = np.asarray(lidar_y, dtype=float)
        lidar_c = np.asarray(lidar_c, dtype=float)
        point_count = min(len(lidar_x), len(lidar_y))
        lidar_x = lidar_x[:point_count]
        lidar_y = lidar_y[:point_count]
        confidence = (
            lidar_c[:point_count]
            if len(lidar_c) >= point_count
            else np.zeros(point_count)
        )
        confidence = np.clip(confidence, 0, 255) / 255.0
        colors = [
            QColor(
                int(int(BLUE[1:3], 16) * (1 - value) + int(RED[1:3], 16) * value),
                int(int(BLUE[3:5], 16) * (1 - value) + int(RED[3:5], 16) * value),
                int(int(BLUE[5:7], 16) * (1 - value) + int(RED[5:7], 16) * value),
            )
            for value in confidence
        ]
        self.lidar_points.setData(
            x=lidar_x, y=lidar_y, brush=colors, pen=None, size=3, symbol="o"
        )
        arrow_x = math.sin(yaw)
        arrow_y = math.cos(yaw)
        end_x = arrow_x
        end_y = arrow_y
        head_base_x = end_x - arrow_x * 0.18
        head_base_y = end_y - arrow_y * 0.18
        head_side_x = math.cos(yaw) * 0.08
        head_side_y = -math.sin(yaw) * 0.08
        self.heading_line.setData([0, end_x], [0, end_y])
        self.heading_head_left.setData(
            [end_x, head_base_x + head_side_x],
            [end_y, head_base_y + head_side_y],
        )
        self.heading_head_right.setData(
            [end_x, head_base_x - head_side_x],
            [end_y, head_base_y - head_side_y],
        )
        self.yaw_text.setText(f"Yaw = {math.degrees(yaw):.1f}°")


class UwbPlot(SquarePlotWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.setBackground(PANEL)
        self.showGrid(x=True, y=True, alpha=0.25)
        self.setLabel("bottom", "X", units="m")
        self.setLabel("left", "Y", units="m")
        self.setMouseEnabled(False, False)
        self.setXRange(0, PLOT_SPAN, padding=0)
        self.setYRange(0, PLOT_SPAN, padding=0)
        self.setAspectLocked(True, ratio=1)
        self.disableAutoRange()
        self.raw_path = self.plot(
            [], [], pen=pg.mkPen(AMBER, width=1.5, style=Qt.PenStyle.DashLine)
        )
        self.filtered_path = self.plot([], [], pen=pg.mkPen(BLUE, width=2))
        self.anchor_markers = self.plot(
            [float(anchor[0]) for anchor in anchors],
            [float(anchor[1]) for anchor in anchors],
            pen=None,
            symbol="o",
            symbolSize=6,
            symbolBrush=TEXT,
            symbolPen=pg.mkPen(BORDER),
        )
        self.anchor_circles = [
            self.plot([], [], pen=pg.mkPen(RED, width=0.8))
            for _ in range(4)
        ]
        self.current_position = self.plot(
            [], [], pen=None, symbol="o", symbolSize=8,
            symbolBrush=RED, symbolPen=pg.mkPen(RED)
        )

    def update_data(self, distances, raw_x, raw_y, filtered_x, filtered_y, x, y):
        self.raw_path.setData(raw_x, raw_y)
        self.filtered_path.setData(filtered_x, filtered_y)
        angles = np.linspace(0, 2 * np.pi, 65)
        for index, circle in enumerate(self.anchor_circles):
            radius = float(distances[index]) if index < len(distances) else 0.0
            anchor_x = float(anchors[index][0])
            anchor_y = float(anchors[index][1])
            circle.setData(
                anchor_x + radius * np.cos(angles),
                anchor_y + radius * np.sin(angles),
            )
        self.current_position.setData([x], [y])


class CameraPlaceholder(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("cameraPanel")
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Expanding,
        )
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        label = QLabel("Camera feed")
        label.setObjectName("cameraText")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(label)

    def sizeHint(self):
        return QSize(300, 300)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UWB / IMU / Lidar Dashboard")
        self.setMinimumSize(1100, 700)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.shared_data = DisplayData()
        self.stop_event = threading.Event()
        self.held = set()
        self.last_command = ""
        self.control_sequence = 0
        self.raw_x = []
        self.raw_y = []
        self.filtered_x = []
        self.filtered_y = []
        self.latest_x = 0.0
        self.latest_y = 0.0
        self.heading_degrees = 0.0
        self.yaw = 0.0
        self.pc_ip = self._get_local_ip()

        self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver_socket.bind(("0.0.0.0", UDP_PORT))
        self.receiver_socket.settimeout(0.25)
        self.control_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._build_ui()
        self.receiver = threading.Thread(
            target=receiver_loop,
            args=(self.shared_data, self.stop_event, self.receiver_socket),
            daemon=True,
        )
        self.receiver.start()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_dashboard)
        self.update_timer.start(33)
        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.send_held_commands)
        self.control_timer.start(round(1000 / CONTROL_RATE_HZ))

    @staticmethod
    def _get_local_ip():
        address_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            address_socket.connect((PI_IP, CONTROL_PORT))
            return address_socket.getsockname()[0]
        except OSError:
            return "--"
        finally:
            address_socket.close()

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 8)
        root.setSpacing(8)

        top_section = QWidget()
        top_section.setFixedHeight(190)
        top_bar = QHBoxLayout(top_section)
        top_bar.setContentsMargins(0, 0, 0, 0)
        top_bar.setSpacing(10)

        compass_panel = QFrame()
        compass_panel.setObjectName("panel")
        compass_panel.setMinimumWidth(190)
        compass_layout = QVBoxLayout(compass_panel)
        compass_layout.setContentsMargins(8, 4, 8, 4)
        compass_layout.setSpacing(0)
        self.compass = CompassWidget()
        compass_layout.addWidget(self.compass, 0, Qt.AlignmentFlag.AlignHCenter)
        self.heading_label = QLabel("Heading 0.0°")
        self.heading_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        compass_layout.addWidget(self.heading_label)
        top_bar.addWidget(compass_panel, 0)

        acceleration_panel = QFrame()
        acceleration_panel.setObjectName("panel")
        acceleration_panel.setFixedWidth(300)
        acceleration_layout = QVBoxLayout(acceleration_panel)
        acceleration_layout.setContentsMargins(8, 4, 8, 6)
        acceleration_layout.setSpacing(2)
        acceleration_title = QLabel("Acceleration")
        acceleration_title.setObjectName("panelTitle")
        acceleration_layout.addWidget(acceleration_title)
        self.acceleration_plot = pg.PlotWidget()
        self.acceleration_plot.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.acceleration_plot.setBackground(PANEL)
        self.acceleration_plot.showGrid(x=True, y=True, alpha=0.25)
        self.acceleration_plot.setLabel("bottom", "Sample")
        self.acceleration_plot.addLegend()
        self.acceleration_x_line = self.acceleration_plot.plot(
            [], [], pen=pg.mkPen(BLUE, width=1.5), name="x acceleration"
        )
        self.acceleration_y_line = self.acceleration_plot.plot(
            [], [], pen=pg.mkPen(RED, width=1.5), name="y acceleration"
        )
        acceleration_layout.addWidget(self.acceleration_plot, 1)
        top_bar.addWidget(acceleration_panel, 0)

        top_bar.addStretch(1)
        status_block = QVBoxLayout()
        status_block.setSpacing(3)
        status_block.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.arm_button = QPushButton("DISARMED")
        self.arm_button.setObjectName("armButton")
        self.arm_button.setCheckable(True)
        self.arm_button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.arm_button.toggled.connect(self._set_armed)
        status_block.addWidget(self.arm_button, 0, Qt.AlignmentFlag.AlignRight)
        shortcut_label = QLabel("Space to arm / disarm")
        shortcut_label.setObjectName("mutedText")
        status_block.addWidget(shortcut_label, 0, Qt.AlignmentFlag.AlignRight)
        self.link_label = QLabel("Link: -- loss | -- Hz | -- dBm")
        self.link_label.setObjectName("linkStatus")
        self.link_label.setProperty("state", "muted")
        status_block.addWidget(self.link_label, 0, Qt.AlignmentFlag.AlignRight)
        ip_label = QLabel(f"Pi: {PI_IP} | PC: {self.pc_ip}")
        ip_label.setObjectName("mutedText")
        status_block.addWidget(ip_label, 0, Qt.AlignmentFlag.AlignRight)
        self.sender_label = QLabel("Waiting for data")
        self.sender_label.setObjectName("waitingStatus")
        status_block.addWidget(self.sender_label, 0, Qt.AlignmentFlag.AlignRight)
        self.command_label = QLabel("Command: idle")
        self.command_label.setObjectName("mutedText")
        status_block.addWidget(self.command_label, 0, Qt.AlignmentFlag.AlignRight)
        top_bar.addLayout(status_block, 0)
        root.addWidget(top_section, 0)

        bottom_section = QWidget()
        bottom_layout = QHBoxLayout(bottom_section)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(10)

        lidar_panel = QFrame()
        lidar_panel.setObjectName("panel")
        lidar_panel.setMinimumSize(300, 300)
        lidar_layout = QVBoxLayout(lidar_panel)
        lidar_layout.setContentsMargins(10, 8, 10, 10)
        lidar_layout.setSpacing(4)
        lidar_title = QLabel("Lidar")
        lidar_title.setObjectName("panelTitle")
        lidar_layout.addWidget(lidar_title)
        self.lidar_plot = LidarPlot()
        lidar_layout.addWidget(self.lidar_plot, 1)
        bottom_layout.addWidget(lidar_panel, 1)

        uwb_panel = QFrame()
        uwb_panel.setObjectName("panel")
        uwb_panel.setMinimumSize(300, 300)
        uwb_layout = QVBoxLayout(uwb_panel)
        uwb_layout.setContentsMargins(10, 8, 10, 10)
        uwb_layout.setSpacing(4)
        uwb_title = QLabel("UWB position")
        uwb_title.setObjectName("panelTitle")
        uwb_layout.addWidget(uwb_title)
        self.uwb_plot = UwbPlot()
        uwb_layout.addWidget(self.uwb_plot, 1)
        bottom_layout.addWidget(uwb_panel, 1)

        camera_panel = QFrame()
        camera_panel.setObjectName("panel")
        camera_panel.setMinimumSize(300, 300)
        camera_layout = QVBoxLayout(camera_panel)
        camera_layout.setContentsMargins(10, 8, 10, 10)
        camera_layout.setSpacing(4)
        camera_title = QLabel("Camera")
        camera_title.setObjectName("panelTitle")
        camera_layout.addWidget(camera_title)
        self.camera_placeholder = CameraPlaceholder()
        camera_layout.addWidget(self.camera_placeholder, 1)
        bottom_layout.addWidget(camera_panel, 1)
        root.addWidget(bottom_section, 1)
        self.setCentralWidget(central)

        status = self.statusBar()
        status.setObjectName("dashboardStatus")
        self.position_status = QLabel("UWB: --, -- m")
        self.anchor_status = QLabel("Anchors: 4/4")
        self.packet_status = QLabel("Packets: 0")
        status.addWidget(self.position_status)
        status.addWidget(self.anchor_status)
        status.addWidget(self.packet_status)

    def _set_armed(self, armed):
        self.arm_button.setText("ARMED" if armed else "DISARMED")
        self.arm_button.setProperty("armed", "true" if armed else "false")
        self.arm_button.style().unpolish(self.arm_button)
        self.arm_button.style().polish(self.arm_button)
        if not armed:
            self._clear_held()
            self._send_control_state()

    def _clear_held(self):
        was_armed = self.arm_button.isChecked()
        self.held.clear()
        self._update_command_label()
        if was_armed:
            self._send_control_state()

    def _update_command_label(self):
        command = "+".join(
            key.decode("ascii")
            for key in (b"L", b"R", b"U", b"D", b"w", b"s", b"d", b"a")
            if key in self.held
        )
        if command == self.last_command:
            return
        self.last_command = command
        self.command_label.setText(f"Command: {command or 'idle'}")
        print(command or "released", flush=True)
        if self.arm_button.isChecked():
            self._send_control_state()

    def send_held_commands(self):
        if not self.arm_button.isChecked():
            return
        self._send_control_state()

    def _send_control_state(self):
        keys = {
            CONTROL_KEYS[key]
            for key in self.held
        }
        packet = pack_control_packet(
            self.control_sequence,
            time.perf_counter(),
            keys,
        )
        try:
            self.control_socket.sendto(packet, (PI_IP, CONTROL_PORT))
        except OSError as exc:
            print(f"control UDP send failed: {exc}", flush=True)
        self.control_sequence = (self.control_sequence + 1) & 0xFFFFFFFF

    def _sequence_loss(self, sequences):
        if not sequences:
            return None
        lost = 0
        expected = 0
        previous = sequences[0]
        for sequence in list(sequences)[1:]:
            difference = (sequence - previous) & 0xFFFFFFFF
            if 0 < difference < 0x80000000:
                lost += difference - 1
                expected += difference
                previous = sequence
        return lost / expected if expected else 0.0

    def _update_link_status(
        self, last_packet_time, arrival_times, sequences, seq, rssi_dbm
    ):
        now = time.monotonic()
        if last_packet_time is None or now - last_packet_time >= 1.0:
            self.link_label.setText("LINK LOST")
            self.link_label.setProperty("state", "red")
            self.link_label.style().unpolish(self.link_label)
            self.link_label.style().polish(self.link_label)
            return

        loss = self._sequence_loss(sequences) if seq is not None else None
        rate = (
            (len(arrival_times) - 1) / (arrival_times[-1] - arrival_times[0])
            if len(arrival_times) > 1 and arrival_times[-1] > arrival_times[0]
            else None
        )
        loss_text = f"{loss * 100:.0f}%" if loss is not None else "--"
        rate_text = f"{rate:.0f} Hz" if rate is not None else "-- Hz"
        rssi_text = f"{rssi_dbm:.0f} dBm" if rssi_dbm is not None else "-- dBm"
        self.link_label.setText(
            f"Link: {loss_text} loss | {rate_text} | {rssi_text}"
        )
        if loss is None or rssi_dbm is None:
            state = "muted"
        elif loss < 0.02 and rssi_dbm > -60:
            state = "green"
        elif loss < 0.10 and rssi_dbm > -70:
            state = "amber"
        else:
            state = "red"
        self.link_label.setProperty("state", state)
        self.link_label.style().unpolish(self.link_label)
        self.link_label.style().polish(self.link_label)

    def _fit_square_plots(self):
        for plot in (self.lidar_plot, self.uwb_plot):
            view_box = plot.getViewBox().geometry()
            if view_box.width() <= 0 or view_box.height() <= 0:
                continue
            height_adjustment = round(view_box.width() - view_box.height())
            if height_adjustment:
                plot.setFixedHeight(
                    max(1, plot.height() + height_adjustment)
                )

    def update_dashboard(self):
        self._fit_square_plots()
        with self.shared_data.lock:
            lidar_x = self.shared_data.lidar_x.copy()
            lidar_y = self.shared_data.lidar_y.copy()
            lidar_c = self.shared_data.lidar_c.copy()
            distances = self.shared_data.distances.copy()
            ax_buffer = list(self.shared_data.ax_buffer)
            ay_buffer = list(self.shared_data.ay_buffer)
            uwb_x = self.shared_data.uwb_x
            uwb_y = self.shared_data.uwb_y
            heading = self.shared_data.heading
            raw_yaw = self.shared_data.yaw
            packet_count = self.shared_data.packet_count
            last_packet_time = self.shared_data.last_packet_time
            arrival_times = list(self.shared_data.arrival_times)
            sequences = list(self.shared_data.sequence_history)
            seq = self.shared_data.seq
            rssi_dbm = self.shared_data.rssi_dbm
            sender_address = self.shared_data.sender_address

        self._update_link_status(
            last_packet_time, arrival_times, sequences, seq, rssi_dbm
        )
        if last_packet_time is None:
            return

        self.raw_x.append(uwb_x)
        self.raw_y.append(uwb_y)
        filtered_x, filtered_y = filter_position(uwb_x, uwb_y)
        self.filtered_x.append(filtered_x)
        self.filtered_y.append(filtered_y)
        self.latest_x = uwb_x
        self.latest_y = uwb_y
        self.heading_degrees = display_heading(math.radians(heading))
        self.yaw = (raw_yaw - (np.pi / 2) + (2 * np.pi)) % (2 * np.pi)

        self.acceleration_x_line.setData(
            np.arange(len(ax_buffer)), ax_buffer
        )
        self.acceleration_y_line.setData(
            np.arange(len(ay_buffer)), ay_buffer
        )
        sample_count = max(len(ax_buffer), len(ay_buffer))
        if sample_count:
            self.acceleration_plot.setXRange(0, sample_count, padding=0)
            y_min = min(
                min(ax_buffer) if ax_buffer else float("inf"),
                min(ay_buffer) if ay_buffer else float("inf"),
            ) - 0.25
            y_max = max(
                max(ax_buffer) if ax_buffer else float("-inf"),
                max(ay_buffer) if ay_buffer else float("-inf"),
            ) + 0.25
            self.acceleration_plot.setYRange(y_min, y_max, padding=0)

        self.compass.set_heading(self.heading_degrees)
        self.heading_label.setText(f"Heading {self.heading_degrees:.1f}°")
        self.lidar_plot.update_data(
            lidar_x,
            lidar_y,
            lidar_c,
            self.yaw,
        )
        self.uwb_plot.update_data(
            distances,
            self.raw_x,
            self.raw_y,
            self.filtered_x,
            self.filtered_y,
            uwb_x,
            uwb_y,
        )
        self.position_status.setText(f"UWB: {uwb_x:.2f}, {uwb_y:.2f} m")
        self.packet_status.setText(f"Packets: {packet_count}")
        if sender_address is not None:
            self.sender_label.setText(
                f"Receiving from {sender_address[0]}:{sender_address[1]}"
            )
            self.sender_label.setObjectName("connectedStatus")
            self.sender_label.style().unpolish(self.sender_label)
            self.sender_label.style().polish(self.sender_label)

    def set_camera_frame(self, qimage):
        pass

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() == Qt.Key.Key_Space:
            self._clear_held()
            self.arm_button.setChecked(not self.arm_button.isChecked())
            event.accept()
            return
        key_map = {
            Qt.Key.Key_Left: b"L",
            Qt.Key.Key_Right: b"R",
            Qt.Key.Key_Up: b"U",
            Qt.Key.Key_Down: b"D",
            Qt.Key.Key_W: b"w",
            Qt.Key.Key_S: b"s",
            Qt.Key.Key_D: b"d",
            Qt.Key.Key_A: b"a",
        }
        key = key_map.get(event.key())
        if key is not None and self.arm_button.isChecked():
            self.held.add(key)
            self._update_command_label()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        key_map = {
            Qt.Key.Key_Left: b"L",
            Qt.Key.Key_Right: b"R",
            Qt.Key.Key_Up: b"U",
            Qt.Key.Key_Down: b"D",
            Qt.Key.Key_W: b"w",
            Qt.Key.Key_S: b"s",
            Qt.Key.Key_D: b"d",
            Qt.Key.Key_A: b"a",
        }
        key = key_map.get(event.key())
        if key is not None and key in self.held:
            self.held.remove(key)
            self._update_command_label()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        self._clear_held()
        super().focusOutEvent(event)

    def closeEvent(self, event):
        self.stop_event.set()
        self.update_timer.stop()
        self.control_timer.stop()
        self.receiver_socket.close()
        self.control_socket.close()
        self.receiver.join()
        event.accept()


def main():
    app = QApplication(sys.argv)
    pg.setConfigOption("background", PANEL)
    pg.setConfigOption("foreground", TEXT)
    app.setStyleSheet(
        f"""
        QWidget {{
            background: {WINDOW};
            color: {TEXT};
            font-size: 13px;
        }}
        QFrame#panel {{
            background: {PANEL};
            border: 1px solid {BORDER};
            border-radius: 8px;
        }}
        QLabel#panelTitle, QLabel#mutedText {{
            color: {MUTED};
        }}
        QLabel#panelTitle {{
            font-size: 11px;
        }}
        QLabel#linkStatus[state="green"] {{ color: {GREEN}; }}
        QLabel#linkStatus[state="amber"] {{ color: {AMBER}; }}
        QLabel#linkStatus[state="red"] {{ color: {RED}; }}
        QLabel#linkStatus[state="muted"] {{ color: {MUTED}; }}
        QPushButton#armButton {{
            background: #4a4d54;
            border: 1px solid {BORDER};
            border-radius: 6px;
            padding: 7px 14px;
            font-weight: bold;
        }}
        QPushButton#armButton[armed="true"] {{ background: {GREEN}; }}
        QPushButton#armButton:hover {{ border-color: {TEXT}; }}
        QFrame#cameraPanel {{
            background: {PANEL};
            border: 2px dashed {BORDER};
            border-radius: 8px;
        }}
        QLabel#cameraText {{ color: {MUTED}; font-size: 16px; }}
        QStatusBar {{
            background: {PANEL};
            border-top: 1px solid {BORDER};
        }}
        QStatusBar::item {{ border: none; padding: 0 10px; }}
        QLabel#waitingStatus {{ color: {MUTED}; }}
        QLabel#connectedStatus {{ color: {GREEN}; }}
        """
    )
    window = MainWindow()
    window.show()
    window.setFocus()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
