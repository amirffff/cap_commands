"""
Motor Monitor - Real-time dashboard for 2-axis motor position & speed.

A high-performance PyQt5 + pyqtgraph application that polls both axes of the
controller in a background thread and displays live position/speed plots and
big readouts. The read frequency, packet delay, history length, per-channel
enables and other settings are all adjustable at runtime.

Usage:
    pip install -r requirements.txt
    python motor_monitor.py
"""

from __future__ import annotations

import csv
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QSizePolicy, QSpinBox,
    QStatusBar, QVBoxLayout, QWidget,
)

import pyqtgraph as pg

from motor_driver import Controller, MotorControl


# ===========================================================================
# Colors / theme
# ===========================================================================
BG_DARK   = "#1e1e2e"
BG_MID    = "#181825"
BG_PANEL  = "#252537"
BORDER    = "#45475a"
TEXT      = "#cdd6f4"
TEXT_DIM  = "#a6adc8"
ACCENT    = "#89b4fa"
GREEN     = "#a6e3a1"
RED       = "#f38ba8"
YELLOW    = "#f9e2af"
PURPLE    = "#cba6f7"

AXIS1_COLOR = "#89b4fa"   # blue
AXIS2_COLOR = "#fab387"   # orange


DARK_STYLESHEET = f"""
QMainWindow {{ background-color: {BG_DARK}; }}
QWidget {{
    background-color: {BG_DARK};
    color: {TEXT};
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 13px;
}}
QGroupBox {{
    background-color: {BG_PANEL};
    border: 1px solid {BORDER};
    border-radius: 8px;
    margin-top: 14px;
    padding: 14px 10px 10px 10px;
    font-weight: bold;
    color: {ACCENT};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
}}
QPushButton {{
    background-color: #313244;
    color: {TEXT};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 6px 14px;
    font-weight: 500;
    min-height: 26px;
}}
QPushButton:hover {{ background-color: #45475a; border-color: {ACCENT}; }}
QPushButton:pressed {{ background-color: #585b70; }}
QPushButton:disabled {{ background-color: {BG_DARK}; color: #585b70; border-color: #313244; }}
QPushButton[class="accent"]  {{ background-color: {ACCENT}; color: {BG_DARK}; font-weight: bold; }}
QPushButton[class="success"] {{ background-color: {GREEN};  color: {BG_DARK}; font-weight: bold; }}
QPushButton[class="danger"]  {{ background-color: {RED};    color: {BG_DARK}; font-weight: bold; }}
QPushButton[class="warning"] {{ background-color: {YELLOW}; color: {BG_DARK}; font-weight: bold; }}
QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {{
    background-color: #313244;
    border: 1px solid {BORDER};
    border-radius: 5px;
    padding: 4px 6px;
    color: {TEXT};
    min-height: 22px;
}}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {{
    border-color: {ACCENT};
}}
QComboBox QAbstractItemView {{
    background-color: #313244;
    color: {TEXT};
    selection-background-color: #45475a;
    border: 1px solid {BORDER};
}}
QCheckBox {{ spacing: 6px; }}
QCheckBox::indicator {{
    width: 16px; height: 16px;
    border: 1px solid {BORDER}; border-radius: 4px;
    background-color: #313244;
}}
QCheckBox::indicator:checked {{ background-color: {ACCENT}; border-color: {ACCENT}; }}
QStatusBar {{
    background-color: {BG_MID};
    color: {TEXT_DIM};
    border-top: 1px solid #313244;
}}
QLabel[class="title"] {{ font-size: 14px; font-weight: bold; color: {PURPLE}; }}
QLabel[class="caption"] {{ color: {TEXT_DIM}; font-size: 11px; }}
QLabel[class="big"] {{
    font-family: "Consolas", "Courier New", monospace;
    font-size: 32px; font-weight: bold;
    background-color: {BG_MID};
    border: 1px solid {BORDER};
    border-radius: 6px;
    padding: 4px 10px;
}}
QLabel[class="unit"] {{ color: {TEXT_DIM}; font-size: 11px; }}
"""


# ===========================================================================
# Fast packet-level read helper (bypasses get_position/get_speed prints + sleep)
# ===========================================================================
@dataclass
class Sample:
    """A single read sample for both axes."""
    t: float                           # seconds since monitor start
    pos: list                          # [axis1_pos_or_None, axis2_pos_or_None]
    spd: list                          # [axis1_spd_or_None, axis2_spd_or_None]


class FastReader:
    """Build packets manually and call send_and_receive with a custom delay.

    Avoids the print() calls in MotorControl.get_position/get_speed and lets
    us set the per-packet sleep_time (the gap between TX and RX). This is what
    determines how fast we can poll the controller.
    """

    def __init__(self, motor: MotorControl):
        self.motor = motor
        # Pre-build read packets once - they never change for the same axis.
        self._pos_packet = motor._build_packet(motor.command_codes['read_position_load'])
        self._spd_packet = motor._build_packet(motor.command_codes['read_speed'])

    def read_position(self, sleep_time: float) -> Optional[float]:
        try:
            r = self.motor.driver.send_and_receive(self._pos_packet, sleep_time=sleep_time)
            if r and len(r) >= 11:
                return self.motor._decode_data(r[7:11], 'float32')
        except Exception:
            return None
        return None

    def read_speed(self, sleep_time: float) -> Optional[float]:
        try:
            r = self.motor.driver.send_and_receive(self._spd_packet, sleep_time=sleep_time)
            if r and len(r) >= 11:
                return self.motor._decode_data(r[7:11], 'float32')
        except Exception:
            return None
        return None


# ===========================================================================
# Background reader thread
# ===========================================================================
class ReaderThread(QThread):
    """Polls both axes in a loop and emits samples to the GUI.

    All adjustable settings are simple attributes - the GUI just sets them
    directly; the read loop picks them up on the next iteration.
    """

    sample = pyqtSignal(object)        # Sample
    rate = pyqtSignal(float)           # actual Hz, emitted ~2/s
    error = pyqtSignal(str)

    def __init__(self, readers: list, parent=None):
        super().__init__(parent)
        self._readers: list[FastReader] = readers
        self._running = False
        self._t0 = time.perf_counter()

        # User-adjustable settings (just assign from the GUI thread).
        self.target_period = 0.05        # seconds between cycles -> 20 Hz
        self.packet_sleep = 0.005        # sleep_time for each TCP request (s)
        self.read_position = [True, True]
        self.read_speed    = [True, True]
        self.paused = False

    def stop(self):
        self._running = False

    def run(self):
        self._running = True
        self._t0 = time.perf_counter()
        next_t = self._t0
        last_rate_emit = self._t0
        cycles_since_emit = 0

        while self._running:
            cycle_start = time.perf_counter()

            if not self.paused:
                pos_vals = [None, None]
                spd_vals = [None, None]
                for i, rdr in enumerate(self._readers):
                    if self.read_position[i]:
                        pos_vals[i] = rdr.read_position(self.packet_sleep)
                    if self.read_speed[i]:
                        spd_vals[i] = rdr.read_speed(self.packet_sleep)

                t = cycle_start - self._t0
                self.sample.emit(Sample(t=t, pos=pos_vals, spd=spd_vals))
                cycles_since_emit += 1

            # Maintain target frequency without drift.
            next_t += self.target_period
            now = time.perf_counter()
            sleep_for = next_t - now
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # We fell behind: reset deadline to "now" so we don't burn CPU
                # catching up indefinitely.
                next_t = now

            # Emit measured rate ~twice per second.
            if now - last_rate_emit >= 0.5:
                hz = cycles_since_emit / (now - last_rate_emit) if cycles_since_emit else 0.0
                self.rate.emit(hz)
                cycles_since_emit = 0
                last_rate_emit = now


# ===========================================================================
# Live history buffer
# ===========================================================================
class History:
    """Bounded circular buffer of (t, pos1, pos2, spd1, spd2)."""

    def __init__(self, maxlen: int = 5000):
        self.maxlen = maxlen
        self.t = deque(maxlen=maxlen)
        self.pos = [deque(maxlen=maxlen), deque(maxlen=maxlen)]
        self.spd = [deque(maxlen=maxlen), deque(maxlen=maxlen)]

    def append(self, s: Sample):
        self.t.append(s.t)
        for i in range(2):
            self.pos[i].append(s.pos[i] if s.pos[i] is not None else np.nan)
            self.spd[i].append(s.spd[i] if s.spd[i] is not None else np.nan)

    def clear(self):
        self.t.clear()
        for d in self.pos: d.clear()
        for d in self.spd: d.clear()

    def resize(self, maxlen: int):
        self.maxlen = maxlen
        self.t = deque(self.t, maxlen=maxlen)
        self.pos = [deque(d, maxlen=maxlen) for d in self.pos]
        self.spd = [deque(d, maxlen=maxlen) for d in self.spd]

    def arrays(self):
        t = np.fromiter(self.t, dtype=np.float64, count=len(self.t))
        pos = [np.fromiter(self.pos[i], dtype=np.float64, count=len(self.pos[i])) for i in range(2)]
        spd = [np.fromiter(self.spd[i], dtype=np.float64, count=len(self.spd[i])) for i in range(2)]
        return t, pos, spd


# ===========================================================================
# Main window
# ===========================================================================
class MotorMonitor(QMainWindow):
    DEFAULT_HOST = "192.168.10.86"
    DEFAULT_PORT = 4949

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Monitor - 2-Axis Live Dashboard")
        self.setMinimumSize(1200, 780)
        self.resize(1400, 880)

        # -- State -----------------------------------------------------------
        self.controller: Optional[Controller] = None
        self.motors: list[Optional[MotorControl]] = [None, None]
        self.reader: Optional[ReaderThread] = None
        self.history = History(maxlen=5000)
        self._last_sample: Optional[Sample] = None
        self._sample_count = 0
        self._error_count = 0

        # -- pyqtgraph global config (fast rendering) ------------------------
        pg.setConfigOptions(
            antialias=False,         # faster
            background=BG_MID,
            foreground=TEXT,
            useOpenGL=False,         # set True if you have a discrete GPU & want more FPS
        )

        # -- Build UI --------------------------------------------------------
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 4)
        root.setSpacing(8)

        root.addWidget(self._build_top_bar())

        body = QHBoxLayout()
        body.setSpacing(8)
        body.addWidget(self._build_side_panel(), 0)
        body.addLayout(self._build_plot_area(), 1)
        root.addLayout(body, 1)

        # GUI redraw timer (decoupled from sample rate => smooth UI).
        self._redraw_timer = QTimer(self)
        self._redraw_timer.timeout.connect(self._redraw_plots)
        self._redraw_timer.start(33)   # ~30 FPS

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._set_status("Disconnected.")
        self._update_ui_state()

    # ------------------------------------------------------------------
    # Top bar: connection + monitoring controls
    # ------------------------------------------------------------------
    def _build_top_bar(self) -> QWidget:
        grp = QGroupBox("Connection & Monitoring")
        lay = QHBoxLayout(grp)
        lay.setSpacing(12)

        lay.addWidget(QLabel("Host:"))
        self.host_input = QLineEdit(self.DEFAULT_HOST)
        self.host_input.setFixedWidth(150)
        lay.addWidget(self.host_input)

        lay.addWidget(QLabel("Port:"))
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535); self.port_input.setValue(self.DEFAULT_PORT)
        self.port_input.setFixedWidth(80)
        lay.addWidget(self.port_input)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setProperty("class", "success")
        self.btn_connect.clicked.connect(self._on_connect)
        lay.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setProperty("class", "danger")
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        lay.addWidget(self.btn_disconnect)

        sep = QFrame(); sep.setFrameShape(QFrame.VLine); sep.setStyleSheet(f"color:{BORDER};")
        lay.addWidget(sep)

        self.btn_start = QPushButton("Start")
        self.btn_start.setProperty("class", "accent")
        self.btn_start.clicked.connect(self._on_start)
        lay.addWidget(self.btn_start)

        self.btn_pause = QPushButton("Pause")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause_toggled)
        lay.addWidget(self.btn_pause)

        self.btn_stop = QPushButton("Stop")
        self.btn_stop.setProperty("class", "warning")
        self.btn_stop.clicked.connect(self._on_stop)
        lay.addWidget(self.btn_stop)

        lay.addStretch()

        self.lbl_rate = QLabel("0.0 Hz")
        self.lbl_rate.setProperty("class", "title")
        self.lbl_rate.setStyleSheet(f"color:{GREEN}; font-weight: bold; font-size: 14px;")
        lay.addWidget(QLabel("Rate:"))
        lay.addWidget(self.lbl_rate)

        self.lbl_samples = QLabel("0 samples")
        self.lbl_samples.setStyleSheet(f"color:{TEXT_DIM};")
        lay.addWidget(self.lbl_samples)

        return grp

    # ------------------------------------------------------------------
    # Side panel: settings & big readouts
    # ------------------------------------------------------------------
    def _build_side_panel(self) -> QWidget:
        wrap = QWidget()
        wrap.setFixedWidth(340)
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(8)

        # ---- Big readouts ---------------------------------------------
        v.addWidget(self._build_readout_card(0, AXIS1_COLOR))
        v.addWidget(self._build_readout_card(1, AXIS2_COLOR))

        # ---- Settings -------------------------------------------------
        grp = QGroupBox("Read Settings")
        g = QGridLayout(grp); g.setSpacing(8)

        r = 0
        g.addWidget(QLabel("Read frequency:"), r, 0)
        self.spin_hz = QDoubleSpinBox()
        self.spin_hz.setRange(0.5, 200.0); self.spin_hz.setDecimals(1)
        self.spin_hz.setSingleStep(1.0); self.spin_hz.setValue(20.0)
        self.spin_hz.setSuffix(" Hz")
        self.spin_hz.valueChanged.connect(self._apply_settings_to_reader)
        g.addWidget(self.spin_hz, r, 1)
        r += 1

        g.addWidget(QLabel("Packet delay:"), r, 0)
        self.spin_delay = QDoubleSpinBox()
        self.spin_delay.setRange(0.0, 50.0); self.spin_delay.setDecimals(1)
        self.spin_delay.setSingleStep(0.5); self.spin_delay.setValue(5.0)
        self.spin_delay.setSuffix(" ms")
        self.spin_delay.setToolTip(
            "Pause between sending a request and reading the response.\n"
            "Lower = faster polling. If you see errors, increase it."
        )
        self.spin_delay.valueChanged.connect(self._apply_settings_to_reader)
        g.addWidget(self.spin_delay, r, 1)
        r += 1

        g.addWidget(QLabel("Buffer size:"), r, 0)
        self.spin_buf = QSpinBox()
        self.spin_buf.setRange(100, 100000); self.spin_buf.setValue(5000)
        self.spin_buf.setSingleStep(500)
        self.spin_buf.setSuffix(" pts")
        self.spin_buf.valueChanged.connect(self._on_buffer_changed)
        g.addWidget(self.spin_buf, r, 1)
        r += 1

        g.addWidget(QLabel("Time window:"), r, 0)
        self.spin_window = QDoubleSpinBox()
        self.spin_window.setRange(1.0, 600.0); self.spin_window.setDecimals(0)
        self.spin_window.setValue(20.0); self.spin_window.setSuffix(" s")
        self.spin_window.setToolTip("How many seconds of history to show on the plot.")
        g.addWidget(self.spin_window, r, 1)
        r += 1

        # Channel enables
        line = QFrame(); line.setFrameShape(QFrame.HLine)
        line.setStyleSheet(f"color:{BORDER};")
        g.addWidget(line, r, 0, 1, 2); r += 1

        g.addWidget(QLabel("Channels:"), r, 0, 1, 2); r += 1
        self.chk_pos = [QCheckBox(f"Axis {i+1} Position") for i in range(2)]
        self.chk_spd = [QCheckBox(f"Axis {i+1} Speed")    for i in range(2)]
        for c in (*self.chk_pos, *self.chk_spd):
            c.setChecked(True)
            c.toggled.connect(self._apply_settings_to_reader)
            c.toggled.connect(self._redraw_plots)
        # Color labels by axis
        self.chk_pos[0].setStyleSheet(f"color:{AXIS1_COLOR};")
        self.chk_spd[0].setStyleSheet(f"color:{AXIS1_COLOR};")
        self.chk_pos[1].setStyleSheet(f"color:{AXIS2_COLOR};")
        self.chk_spd[1].setStyleSheet(f"color:{AXIS2_COLOR};")
        g.addWidget(self.chk_pos[0], r, 0); g.addWidget(self.chk_pos[1], r, 1); r += 1
        g.addWidget(self.chk_spd[0], r, 0); g.addWidget(self.chk_spd[1], r, 1); r += 1
        v.addWidget(grp)

        # ---- Data actions --------------------------------------------
        grp2 = QGroupBox("Data")
        h = QHBoxLayout(grp2)
        self.btn_clear = QPushButton("Clear")
        self.btn_clear.clicked.connect(self._on_clear)
        h.addWidget(self.btn_clear)
        self.btn_save = QPushButton("Save CSV...")
        self.btn_save.clicked.connect(self._on_save_csv)
        h.addWidget(self.btn_save)
        v.addWidget(grp2)

        v.addStretch()
        return wrap

    def _build_readout_card(self, idx: int, color: str) -> QGroupBox:
        axis = idx + 1
        grp = QGroupBox(f"Axis {axis}")
        grp.setStyleSheet(f"QGroupBox {{ color:{color}; }}")
        g = QGridLayout(grp); g.setSpacing(6)

        # Position
        lbl_p_cap = QLabel("Position")
        lbl_p_cap.setProperty("class", "caption")
        lbl_p_val = QLabel("---")
        lbl_p_val.setProperty("class", "big")
        lbl_p_val.setStyleSheet(f"color:{color}; background-color:{BG_MID}; border:1px solid {BORDER}; "
                                f"border-radius:6px; padding:4px 10px; font-family:Consolas; "
                                f"font-size:30px; font-weight:bold;")
        lbl_p_val.setMinimumWidth(160)
        lbl_p_unit = QLabel("deg"); lbl_p_unit.setProperty("class", "unit")

        g.addWidget(lbl_p_cap, 0, 0)
        g.addWidget(lbl_p_val, 1, 0)
        g.addWidget(lbl_p_unit, 1, 1, alignment=Qt.AlignLeft | Qt.AlignBottom)

        # Speed
        lbl_s_cap = QLabel("Speed")
        lbl_s_cap.setProperty("class", "caption")
        lbl_s_val = QLabel("---")
        lbl_s_val.setStyleSheet(f"color:{color}; background-color:{BG_MID}; border:1px solid {BORDER}; "
                                f"border-radius:6px; padding:4px 10px; font-family:Consolas; "
                                f"font-size:30px; font-weight:bold;")
        lbl_s_val.setMinimumWidth(160)
        lbl_s_unit = QLabel("deg/s"); lbl_s_unit.setProperty("class", "unit")

        g.addWidget(lbl_s_cap, 2, 0)
        g.addWidget(lbl_s_val, 3, 0)
        g.addWidget(lbl_s_unit, 3, 1, alignment=Qt.AlignLeft | Qt.AlignBottom)

        if idx == 0:
            self.lbl_pos1, self.lbl_spd1 = lbl_p_val, lbl_s_val
        else:
            self.lbl_pos2, self.lbl_spd2 = lbl_p_val, lbl_s_val
        return grp

    # ------------------------------------------------------------------
    # Plot area
    # ------------------------------------------------------------------
    def _build_plot_area(self) -> QVBoxLayout:
        v = QVBoxLayout(); v.setSpacing(6)

        # Position plot
        self.pos_plot = pg.PlotWidget(title="Position")
        self.pos_plot.setLabel("left", "Position", units="deg")
        self.pos_plot.setLabel("bottom", "Time", units="s")
        self.pos_plot.showGrid(x=True, y=True, alpha=0.25)
        self.pos_plot.addLegend(offset=(10, 10))
        self.pos_plot.setMouseEnabled(x=True, y=True)
        self.pos_curve = [
            self.pos_plot.plot(pen=pg.mkPen(AXIS1_COLOR, width=2), name="Axis 1"),
            self.pos_plot.plot(pen=pg.mkPen(AXIS2_COLOR, width=2), name="Axis 2"),
        ]

        # Speed plot
        self.spd_plot = pg.PlotWidget(title="Speed")
        self.spd_plot.setLabel("left", "Speed", units="deg/s")
        self.spd_plot.setLabel("bottom", "Time", units="s")
        self.spd_plot.showGrid(x=True, y=True, alpha=0.25)
        self.spd_plot.addLegend(offset=(10, 10))
        self.spd_plot.setMouseEnabled(x=True, y=True)
        self.spd_curve = [
            self.spd_plot.plot(pen=pg.mkPen(AXIS1_COLOR, width=2), name="Axis 1"),
            self.spd_plot.plot(pen=pg.mkPen(AXIS2_COLOR, width=2), name="Axis 2"),
        ]

        # Link x axes so they pan/zoom together
        self.spd_plot.setXLink(self.pos_plot)

        for pw in (self.pos_plot, self.spd_plot):
            pw.setBackground(BG_MID)
            pw.getAxis("left").setTextPen(TEXT)
            pw.getAxis("bottom").setTextPen(TEXT)
            pw.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        v.addWidget(self.pos_plot, 1)
        v.addWidget(self.spd_plot, 1)
        return v

    # ------------------------------------------------------------------
    # Connection actions
    # ------------------------------------------------------------------
    def _on_connect(self):
        host = self.host_input.text().strip()
        port = self.port_input.value()
        if not host:
            QMessageBox.warning(self, "Input", "Please provide a host address.")
            return
        self._set_status(f"Connecting to {host}:{port}...")
        QApplication.processEvents()

        ctrl = Controller(host=host, port=port, timeout=3.0)
        if not ctrl.connect():
            QMessageBox.critical(self, "Connection Failed",
                                 f"Could not connect to {host}:{port}.")
            self._set_status("Connection failed.")
            return

        self.controller = ctrl
        self.motors[0] = MotorControl(ctrl, axis_number=1)
        self.motors[1] = MotorControl(ctrl, axis_number=2)
        self._set_status(f"Connected to {host}:{port}.")
        self._update_ui_state()

    def _on_disconnect(self):
        self._on_stop()
        if self.controller:
            self.controller.disconnect()
        self.controller = None
        self.motors = [None, None]
        self._set_status("Disconnected.")
        self._update_ui_state()

    # ------------------------------------------------------------------
    # Monitoring actions
    # ------------------------------------------------------------------
    def _on_start(self):
        if not self.controller or not self.controller.is_connected():
            QMessageBox.warning(self, "Not connected", "Connect to the controller first.")
            return
        if self.reader is not None:
            return  # already running

        readers = [FastReader(self.motors[0]), FastReader(self.motors[1])]
        self.reader = ReaderThread(readers, parent=self)
        self._apply_settings_to_reader()
        self.reader.sample.connect(self._on_sample)
        self.reader.rate.connect(self._on_rate)
        self.reader.error.connect(self._on_error)
        self.history.resize(self.spin_buf.value())
        self._sample_count = 0
        self.reader.start(QThread.HighPriority)
        self._set_status("Monitoring started.")
        self._update_ui_state()

    def _on_stop(self):
        if self.reader is not None:
            self.reader.stop()
            self.reader.wait(2000)
            self.reader = None
        self._set_status("Monitoring stopped.")
        self.btn_pause.setChecked(False)
        self._update_ui_state()

    def _on_pause_toggled(self, paused: bool):
        if self.reader is not None:
            self.reader.paused = paused
        self.btn_pause.setText("Resume" if paused else "Pause")

    def _apply_settings_to_reader(self):
        if self.reader is None:
            return
        hz = max(0.1, self.spin_hz.value())
        self.reader.target_period = 1.0 / hz
        self.reader.packet_sleep = self.spin_delay.value() / 1000.0
        self.reader.read_position = [c.isChecked() for c in self.chk_pos]
        self.reader.read_speed    = [c.isChecked() for c in self.chk_spd]

    def _on_buffer_changed(self, n: int):
        self.history.resize(n)

    def _on_clear(self):
        self.history.clear()
        self._sample_count = 0
        self.lbl_samples.setText("0 samples")
        for curve in (*self.pos_curve, *self.spd_curve):
            curve.setData([], [])

    def _on_save_csv(self):
        if not self.history.t:
            QMessageBox.information(self, "No data", "There is no data to save yet.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Save samples as CSV",
            f"motor_log_{int(time.time())}.csv",
            "CSV files (*.csv)"
        )
        if not path:
            return
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["time_s", "axis1_position_deg", "axis2_position_deg",
                            "axis1_speed_deg_s", "axis2_speed_deg_s"])
                for i in range(len(self.history.t)):
                    w.writerow([
                        f"{self.history.t[i]:.6f}",
                        self.history.pos[0][i],
                        self.history.pos[1][i],
                        self.history.spd[0][i],
                        self.history.spd[1][i],
                    ])
            self._set_status(f"Saved {len(self.history.t)} samples to {path}")
        except Exception as e:
            QMessageBox.critical(self, "Save failed", str(e))

    # ------------------------------------------------------------------
    # Sample handling
    # ------------------------------------------------------------------
    def _on_sample(self, s: Sample):
        self.history.append(s)
        self._last_sample = s
        self._sample_count += 1

        # Big readouts update on every sample (cheap).
        if s.pos[0] is not None:
            self.lbl_pos1.setText(f"{s.pos[0]:+.2f}")
        if s.pos[1] is not None:
            self.lbl_pos2.setText(f"{s.pos[1]:+.2f}")
        if s.spd[0] is not None:
            self.lbl_spd1.setText(f"{s.spd[0]:+.2f}")
        if s.spd[1] is not None:
            self.lbl_spd2.setText(f"{s.spd[1]:+.2f}")
        self.lbl_samples.setText(f"{self._sample_count} samples")

    def _on_rate(self, hz: float):
        self.lbl_rate.setText(f"{hz:.1f} Hz")

    def _on_error(self, msg: str):
        self._error_count += 1
        self._set_status(f"Error: {msg}  (errors so far: {self._error_count})")

    # ------------------------------------------------------------------
    # Plot redraw (decoupled from sample rate)
    # ------------------------------------------------------------------
    def _redraw_plots(self):
        if not self.history.t:
            return
        t, pos, spd = self.history.arrays()

        # Sliding time window
        window = self.spin_window.value()
        if len(t) >= 2 and (t[-1] - t[0]) > window:
            x_min = t[-1] - window
        else:
            x_min = t[0]
        x_max = t[-1]

        for i in range(2):
            if self.chk_pos[i].isChecked():
                self.pos_curve[i].setData(t, pos[i], connect="finite")
            else:
                self.pos_curve[i].setData([], [])
            if self.chk_spd[i].isChecked():
                self.spd_curve[i].setData(t, spd[i], connect="finite")
            else:
                self.spd_curve[i].setData([], [])

        self.pos_plot.setXRange(x_min, x_max, padding=0)
        # speed shares X via setXLink

    # ------------------------------------------------------------------
    # UI helpers
    # ------------------------------------------------------------------
    def _set_status(self, msg: str):
        self.status_bar.showMessage(msg)

    def _update_ui_state(self):
        connected = bool(self.controller and self.controller.is_connected())
        monitoring = self.reader is not None

        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.host_input.setEnabled(not connected)
        self.port_input.setEnabled(not connected)

        self.btn_start.setEnabled(connected and not monitoring)
        self.btn_stop.setEnabled(monitoring)
        self.btn_pause.setEnabled(monitoring)

    def closeEvent(self, event):
        self._on_stop()
        if self.controller:
            self.controller.disconnect()
        event.accept()


# ===========================================================================
# Entry point
# ===========================================================================
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLESHEET)
    app.setFont(QFont("Segoe UI", 10))
    win = MotorMonitor()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
