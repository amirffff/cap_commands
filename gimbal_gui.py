"""
Gimbal Control GUI - PyQt5 Application
Controls a 2-axis gimbal via the MotorControl/Controller classes.
"""
from __future__ import annotations

import json
import math
import os
import sys
import struct
import tempfile
from functools import partial
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QDoubleSpinBox, QComboBox,
    QStatusBar, QTabWidget, QGridLayout, QFrame, QSplitter,
    QMessageBox, QCheckBox, QSpinBox, QLineEdit, QScrollArea,
    QProgressBar, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QUrl, QElapsedTimer
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon

try:
    from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEngineSettings, QWebEnginePage
    WEBENGINE_AVAILABLE = True
except Exception:
    QWebEngineView = None
    QWebEngineSettings = None
    QWebEnginePage = None
    WEBENGINE_AVAILABLE = False

from motor_driver import Controller, MotorControl


# ---------------------------------------------------------------------------
# Stylesheet
# ---------------------------------------------------------------------------
DARK_STYLESHEET = """
QMainWindow {
    background-color: #1e1e2e;
}
QWidget {
    background-color: #1e1e2e;
    color: #cdd6f4;
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 13px;
}
QGroupBox {
    border: 1px solid #45475a;
    border-radius: 8px;
    margin-top: 14px;
    padding: 14px 10px 10px 10px;
    font-weight: bold;
    font-size: 13px;
    color: #89b4fa;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
}
QPushButton {
    background-color: #313244;
    color: #cdd6f4;
    border: 1px solid #45475a;
    border-radius: 6px;
    padding: 7px 18px;
    font-weight: 500;
    min-height: 28px;
}
QPushButton:hover {
    background-color: #45475a;
    border-color: #89b4fa;
}
QPushButton:pressed {
    background-color: #585b70;
}
QPushButton:disabled {
    background-color: #1e1e2e;
    color: #585b70;
    border-color: #313244;
}
QPushButton[class="accent"] {
    background-color: #89b4fa;
    color: #1e1e2e;
    font-weight: bold;
}
QPushButton[class="accent"]:hover {
    background-color: #b4d0fb;
}
QPushButton[class="danger"] {
    background-color: #f38ba8;
    color: #1e1e2e;
    font-weight: bold;
}
QPushButton[class="danger"]:hover {
    background-color: #f5a3b8;
}
QPushButton[class="success"] {
    background-color: #a6e3a1;
    color: #1e1e2e;
    font-weight: bold;
}
QPushButton[class="success"]:hover {
    background-color: #b8eab4;
}
QPushButton[class="warning"] {
    background-color: #fab387;
    color: #1e1e2e;
    font-weight: bold;
}
QPushButton[class="warning"]:hover {
    background-color: #fbc4a0;
}
QDoubleSpinBox, QSpinBox, QLineEdit {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 5px;
    padding: 5px 8px;
    color: #cdd6f4;
    min-height: 24px;
}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus {
    border-color: #89b4fa;
}
QComboBox {
    background-color: #313244;
    border: 1px solid #45475a;
    border-radius: 5px;
    padding: 5px 8px;
    color: #cdd6f4;
    min-height: 24px;
}
QComboBox:hover {
    border-color: #89b4fa;
}
QComboBox::drop-down {
    border: none;
    width: 24px;
}
QComboBox QAbstractItemView {
    background-color: #313244;
    color: #cdd6f4;
    selection-background-color: #45475a;
    border: 1px solid #45475a;
    border-radius: 4px;
}
QTabWidget::pane {
    border: 1px solid #45475a;
    border-radius: 6px;
    top: -1px;
}
QTabBar::tab {
    background-color: #313244;
    color: #a6adc8;
    border: 1px solid #45475a;
    border-bottom: none;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    padding: 8px 20px;
    margin-right: 2px;
    font-weight: 500;
}
QTabBar::tab:selected {
    background-color: #1e1e2e;
    color: #89b4fa;
    border-bottom: 2px solid #89b4fa;
}
QTabBar::tab:hover:!selected {
    background-color: #45475a;
}
QLabel[class="heading"] {
    font-size: 15px;
    font-weight: bold;
    color: #cba6f7;
}
QLabel[class="value"] {
    font-size: 22px;
    font-weight: bold;
    color: #a6e3a1;
    font-family: "Consolas", "Courier New", monospace;
}
QLabel[class="map-value"] {
    font-size: 15px;
    font-weight: bold;
    color: #a6e3a1;
    font-family: "Consolas", "Courier New", monospace;
}
QLabel[class="unit"] {
    color: #6c7086;
    font-size: 11px;
}
QLabel[class="status-good"] { color: #a6e3a1; font-weight: bold; }
QLabel[class="status-warn"] { color: #fab387; font-weight: bold; }
QLabel[class="status-bad"] { color: #f38ba8; font-weight: bold; }
QStatusBar {
    background-color: #181825;
    color: #a6adc8;
    border-top: 1px solid #313244;
    font-size: 12px;
}
QFrame[class="separator"] {
    background-color: #45475a;
    max-height: 1px;
}
QCheckBox {
    spacing: 6px;
}
QCheckBox::indicator {
    width: 18px;
    height: 18px;
    border: 1px solid #45475a;
    border-radius: 4px;
    background-color: #313244;
}
QCheckBox::indicator:checked {
    background-color: #89b4fa;
    border-color: #89b4fa;
}
"""


# ---------------------------------------------------------------------------
# Worker thread for non-blocking motor commands
# ---------------------------------------------------------------------------
class MotorWorker(QThread):
    """Run motor commands in a background thread to keep GUI responsive."""
    command_finished = pyqtSignal(str, bool)   # (description, success)
    position_read = pyqtSignal(int, object)    # (axis_index, value_or_None)
    speed_read = pyqtSignal(int, object)

    def __init__(
        self,
        func,
        *args,
        description="",
        axis_index=0,
        read_type="",
        motors=None,
        read_positions_after=False,
    ):
        super().__init__()
        self._func = func
        self._args = args
        self._desc = description
        self._axis = axis_index
        self._read_type = read_type
        self._motors = list(motors or [])
        self._read_positions_after = read_positions_after

    def run(self):
        try:
            result = self._func(*self._args)
            if self._read_type == "position":
                self.position_read.emit(self._axis, result)
            elif self._read_type == "speed":
                self.speed_read.emit(self._axis, result)
            else:
                success = bool(result)
                self.command_finished.emit(self._desc, success)
                if success and self._read_positions_after:
                    for idx, motor in enumerate(self._motors):
                        if motor is None:
                            continue
                        try:
                            pos = motor.get_position()
                            self.position_read.emit(idx, pos)
                        except Exception:
                            self.position_read.emit(idx, None)
        except Exception as e:
            if self._read_type:
                if self._read_type == "position":
                    self.position_read.emit(self._axis, None)
                else:
                    self.speed_read.emit(self._axis, None)
            else:
                self.command_finished.emit(f"{self._desc} - Error: {e}", False)


class PositionPollWorker(QThread):
    """Read axis positions sequentially on one thread (avoids serial bus collisions)."""
    position_read = pyqtSignal(int, object)
    poll_data_ready = pyqtSignal()

    def __init__(self, motors: list):
        super().__init__()
        self._motors = list(motors)

    def run(self):
        for idx, motor in enumerate(self._motors):
            if motor is None:
                continue
            try:
                value = motor.get_position()
            except Exception:
                value = None
            self.position_read.emit(idx, value)
        self.poll_data_ready.emit()


# ---------------------------------------------------------------------------
# Indicator widget (small coloured circle)
# ---------------------------------------------------------------------------
class StatusIndicator(QLabel):
    """Small coloured dot to indicate status."""
    def __init__(self, size=14, parent=None):
        super().__init__(parent)
        self._size = size
        self.setFixedSize(size, size)
        self.set_color("#585b70")

    def set_color(self, hex_color: str):
        self.setStyleSheet(
            f"background-color: {hex_color}; border-radius: {self._size // 2}px; "
            f"border: 1px solid #45475a;"
        )


# ---------------------------------------------------------------------------
# Main Window
# ---------------------------------------------------------------------------
class GimbalControlGUI(QMainWindow):
    DEFAULT_HOST = "192.168.10.120"
    DEFAULT_PORT = 4949

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Gimbal Motor Control")
        self.setMinimumSize(1400, 850)
        self.resize(1500, 900)

        # ---- State ----------------------------------------------------------
        self.controller: Controller | None = None
        self.motors: list[MotorControl | None] = [None, None]
        self._workers: list[MotorWorker] = []
        self._axis_positions: list[float | None] = [None, None]
        self._axis_positions_last_good: list[float | None] = [None, None]
        self._poll_worker: PositionPollWorker | None = None
        self._map_sync_active = False
        self._hardware_scan_active = False
        self._hardware_scan_params: dict | None = None
        self._map_loaded = False
        self._map_online = False
        self._map_interactive = False
        self._map_engine_name = "Leaflet 2D"
        self._preset_file = os.path.join(os.path.dirname(__file__), "map_presets.json")
        self._map_presets: dict[str, dict[str, float]] = {}
        self._map_tmp_file: str | None = None
        self._map_load_id = 0
        self._map_reload_timer = QTimer(self)
        self._map_reload_timer.setSingleShot(True)
        self._map_reload_timer.setInterval(25000)
        self._map_reload_timer.timeout.connect(self._on_map_reload_timeout)
        self._scan_timer = QTimer(self)
        self._scan_timer.setInterval(50)
        self._scan_timer.timeout.connect(self._tick_scan_demo)
        self._scan_elapsed = QElapsedTimer()
        self._scan_state: dict | None = None
        self._polling_timer = QTimer(self)
        self._polling_timer.timeout.connect(self._poll_positions)
        self._map_overlay_timer = QTimer(self)
        self._map_overlay_timer.setSingleShot(True)
        self._map_overlay_timer.setInterval(60)
        self._map_overlay_timer.timeout.connect(
            lambda: self._refresh_map_overlay(pan_map=False)
        )
        self._speed_poll_counter = 0
        self._POST_MOVE_READ_METHODS = frozenset({
            "set_position", "update", "set_speed", "set_movement_mode",
            "set_movement_type", "SSL_position",
        })

        # ---- Build UI -------------------------------------------------------
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 4)
        root_layout.setSpacing(8)

        # Connection bar
        root_layout.addWidget(self._build_connection_bar())

        # Tabs
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_combined_ops_tab(), "Gimbal & Map")
        self.tabs.addTab(self._build_ssl_tab(), "SSL (Sync Both)")
        self.tabs.addTab(self._build_config_tab(), "Configuration")
        root_layout.addWidget(self.tabs)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._set_status("Disconnected")

        self._load_map_presets()
        self._update_ui_state()
        self._update_map_sync_label()
        self._update_map_live_status()
        self._map_health_timer = QTimer(self)
        self._map_health_timer.timeout.connect(self._poll_map_status)
        self._map_health_timer.start(2500)

    # =====================================================================
    # Connection bar
    # =====================================================================
    def _build_connection_bar(self) -> QWidget:
        grp = QGroupBox("Connection")
        lay = QHBoxLayout(grp)
        lay.setSpacing(12)

        lay.addWidget(QLabel("Host:"))
        self.host_input = QLineEdit(self.DEFAULT_HOST)
        self.host_input.setFixedWidth(160)
        lay.addWidget(self.host_input)

        lay.addWidget(QLabel("Port:"))
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(self.DEFAULT_PORT)
        self.port_input.setFixedWidth(90)
        lay.addWidget(self.port_input)

        self.conn_indicator = StatusIndicator()
        lay.addWidget(self.conn_indicator)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setProperty("class", "success")
        self.btn_connect.clicked.connect(self._on_connect)
        lay.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("Disconnect")
        self.btn_disconnect.setProperty("class", "danger")
        self.btn_disconnect.clicked.connect(self._on_disconnect)
        self.btn_disconnect.setEnabled(False)
        lay.addWidget(self.btn_disconnect)

        lay.addStretch()

        # Polling
        self.chk_poll = QCheckBox("Auto-read positions (map)")
        self.chk_poll.setChecked(False)
        self.chk_poll.toggled.connect(self._toggle_polling)
        lay.addWidget(self.chk_poll)

        self.poll_interval = QSpinBox()
        self.poll_interval.setRange(200, 10000)
        self.poll_interval.setValue(300)
        self.poll_interval.setToolTip(
            "Interval between position reads. Too fast causes unstable readings."
        )
        self.poll_interval.setSuffix(" ms")
        self.poll_interval.setFixedWidth(100)
        self.poll_interval.valueChanged.connect(
            lambda v: self._polling_timer.setInterval(v) if self._polling_timer.isActive() else None
        )
        lay.addWidget(self.poll_interval)

        return grp

    # =====================================================================
    # Motor axis panels (embedded in combined ops tab)
    # =====================================================================
    def _build_motor_controls_widget(self) -> QTabWidget:
        """One tab per axis so controls are not squeezed side-by-side."""
        if not hasattr(self, "axis_panels"):
            self.axis_panels: list[dict] = []

        motor_tabs = QTabWidget()
        motor_tabs.setDocumentMode(True)

        for idx in range(2):
            if idx < len(self.axis_panels):
                panel = self.axis_panels[idx]
            else:
                panel = self._build_axis_panel(idx)
                self.axis_panels.append(panel)

            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.NoFrame)
            scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

            page = QWidget()
            page_lay = QVBoxLayout(page)
            page_lay.setContentsMargins(8, 8, 8, 8)
            page_lay.setSpacing(10)
            page_lay.addWidget(panel["group"])
            page_lay.addStretch(1)
            scroll.setWidget(page)
            motor_tabs.addTab(scroll, f"Axis {idx + 1}")

        return motor_tabs

    def _build_axis_panel(self, idx: int) -> dict:
        axis_num = idx + 1
        grp = QGroupBox(f"Motor {axis_num}")
        lay = QVBoxLayout(grp)
        lay.setSpacing(10)
        widgets: dict = {"group": grp}

        def _hline() -> QFrame:
            line = QFrame()
            line.setFrameShape(QFrame.HLine)
            line.setProperty("class", "separator")
            return line

        def _field_row(label: str, field: QWidget, button: QPushButton | None = None) -> QHBoxLayout:
            row = QHBoxLayout()
            lbl = QLabel(label)
            lbl.setMinimumWidth(100)
            row.addWidget(lbl)
            field.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            row.addWidget(field, 1)
            if button is not None:
                button.setMinimumWidth(52)
                row.addWidget(button)
            return row

        # -- Power / homing ----------------------------------------------------
        power = QHBoxLayout()
        btn_on = QPushButton("Enable")
        btn_on.setProperty("class", "success")
        btn_on.clicked.connect(lambda checked, i=idx: self._cmd(i, "axis_on"))
        btn_off = QPushButton("Disable")
        btn_off.setProperty("class", "danger")
        btn_off.clicked.connect(lambda checked, i=idx: self._cmd(i, "axis_off"))
        btn_reset = QPushButton("Reset")
        btn_reset.setProperty("class", "warning")
        btn_reset.clicked.connect(lambda checked, i=idx: self._cmd(i, "axis_reset"))
        btn_home = QPushButton("Homing")
        btn_home.clicked.connect(lambda checked, i=idx: self._cmd(i, "homing"))
        for btn in (btn_on, btn_off, btn_reset, btn_home):
            power.addWidget(btn)
        widgets["btn_on"] = btn_on
        widgets["btn_off"] = btn_off
        lay.addLayout(power)
        lay.addWidget(_hline())

        # -- Movement mode & type ----------------------------------------------
        combo_mode = QComboBox()
        combo_mode.addItems(["position", "speed"])
        btn_set_mode = QPushButton("Set Mode")
        btn_set_mode.clicked.connect(
            lambda checked, i=idx, cb=combo_mode: self._cmd(i, "set_movement_mode", cb.currentText())
        )
        lay.addLayout(_field_row("Mode:", combo_mode, btn_set_mode))
        widgets["combo_mode"] = combo_mode

        combo_type = QComboBox()
        combo_type.addItems(["absolute", "relative"])
        btn_set_type = QPushButton("Set Type")
        btn_set_type.clicked.connect(
            lambda checked, i=idx, cb=combo_type: self._cmd(i, "set_movement_type", cb.currentText())
        )
        lay.addLayout(_field_row("Type:", combo_type, btn_set_type))
        widgets["combo_type"] = combo_type
        lay.addWidget(_hline())

        # -- Position / speed / acceleration -----------------------------------
        spin_pos = QDoubleSpinBox()
        spin_pos.setRange(-9999.0, 9999.0)
        spin_pos.setDecimals(2)
        spin_pos.setSuffix(" deg")
        btn_go_pos = QPushButton("Go")
        btn_go_pos.setProperty("class", "accent")
        btn_go_pos.setToolTip("Go to position")
        btn_go_pos.clicked.connect(
            lambda checked, i=idx, sp=spin_pos: self._cmd(i, "set_position", sp.value())
        )
        lay.addLayout(_field_row("Position:", spin_pos, btn_go_pos))
        widgets["spin_pos"] = spin_pos

        spin_spd = QDoubleSpinBox()
        spin_spd.setRange(-9999.0, 9999.0)
        spin_spd.setDecimals(2)
        spin_spd.setSuffix(" deg/s")
        btn_set_spd = QPushButton("Set")
        btn_set_spd.setProperty("class", "accent")
        btn_set_spd.setToolTip("Set speed")
        btn_set_spd.clicked.connect(
            lambda checked, i=idx, sp=spin_spd: self._cmd(i, "set_speed", sp.value())
        )
        lay.addLayout(_field_row("Speed:", spin_spd, btn_set_spd))
        widgets["spin_spd"] = spin_spd

        spin_acc = QDoubleSpinBox()
        spin_acc.setRange(0.0, 9999.0)
        spin_acc.setDecimals(2)
        spin_acc.setSuffix(" deg/s²")
        spin_acc.setValue(10.0)
        btn_set_acc = QPushButton("Set")
        btn_set_acc.setToolTip("Set acceleration")
        btn_set_acc.clicked.connect(
            lambda checked, i=idx, sp=spin_acc: self._cmd(i, "set_acceleration", sp.value())
        )
        lay.addLayout(_field_row("Acceleration:", spin_acc, btn_set_acc))
        widgets["spin_acc"] = spin_acc

        btn_update = QPushButton("Update (execute)")
        btn_update.setProperty("class", "accent")
        btn_update.clicked.connect(lambda checked, i=idx: self._cmd(i, "update"))
        lay.addWidget(btn_update)
        lay.addWidget(_hline())

        # -- Readouts ----------------------------------------------------------
        read_pos_row = QHBoxLayout()
        read_pos_row.addWidget(QLabel("Current position:"))
        lbl_pos = QLabel("---")
        lbl_pos.setProperty("class", "value")
        lbl_pos.setMinimumWidth(80)
        read_pos_row.addWidget(lbl_pos, 1)
        btn_read_pos = QPushButton("Read")
        btn_read_pos.clicked.connect(partial(self._read_position, idx))
        read_pos_row.addWidget(btn_read_pos)
        widgets["lbl_pos"] = lbl_pos
        lay.addLayout(read_pos_row)

        read_spd_row = QHBoxLayout()
        read_spd_row.addWidget(QLabel("Current speed:"))
        lbl_spd = QLabel("---")
        lbl_spd.setProperty("class", "value")
        lbl_spd.setMinimumWidth(80)
        read_spd_row.addWidget(lbl_spd, 1)
        btn_read_spd = QPushButton("Read")
        btn_read_spd.clicked.connect(partial(self._read_speed, idx))
        read_spd_row.addWidget(btn_read_spd)
        widgets["lbl_spd"] = lbl_spd
        lay.addLayout(read_spd_row)

        return widgets

    # =====================================================================
    # SSL tab  (simultaneous dual-axis commands)
    # =====================================================================
    def _build_ssl_tab(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setSpacing(14)

        # --- SSL Position ---
        grp_pos = QGroupBox("SSL Position  (move both axes at once)")
        gp = QGridLayout(grp_pos)
        gp.setSpacing(8)

        gp.addWidget(QLabel("Axis 1 Position:"), 0, 0)
        self.ssl_pos1 = QDoubleSpinBox()
        self.ssl_pos1.setRange(-9999, 9999); self.ssl_pos1.setDecimals(2)
        self.ssl_pos1.setSuffix("  deg")
        gp.addWidget(self.ssl_pos1, 0, 1)

        gp.addWidget(QLabel("Axis 2 Position:"), 0, 2)
        self.ssl_pos2 = QDoubleSpinBox()
        self.ssl_pos2.setRange(-9999, 9999); self.ssl_pos2.setDecimals(2)
        self.ssl_pos2.setSuffix("  deg")
        gp.addWidget(self.ssl_pos2, 0, 3)

        gp.addWidget(QLabel("Mode:"), 1, 0)
        self.ssl_pos_mode = QComboBox()
        self.ssl_pos_mode.addItems([
            "0 - Last defined",
            "1 - Both absolute",
            "2 - Both relative",
            "3 - Ax1 abs / Ax2 rel",
            "4 - Ax1 rel / Ax2 abs",
        ])
        self.ssl_pos_mode.setCurrentIndex(1)
        gp.addWidget(self.ssl_pos_mode, 1, 1, 1, 2)

        btn_ssl_pos = QPushButton("Send SSL Position")
        btn_ssl_pos.setProperty("class", "accent")
        btn_ssl_pos.clicked.connect(self._on_ssl_position)
        gp.addWidget(btn_ssl_pos, 1, 3)
        outer.addWidget(grp_pos)

        # --- SSL Speed ---
        grp_spd = QGroupBox("SSL Speed  (set speed for both axes)")
        gs = QGridLayout(grp_spd)
        gs.setSpacing(8)

        gs.addWidget(QLabel("Axis 1 Speed:"), 0, 0)
        self.ssl_spd1 = QDoubleSpinBox()
        self.ssl_spd1.setRange(-9999, 9999); self.ssl_spd1.setDecimals(2)
        self.ssl_spd1.setSuffix("  deg/s")
        gs.addWidget(self.ssl_spd1, 0, 1)

        gs.addWidget(QLabel("Axis 2 Speed:"), 0, 2)
        self.ssl_spd2 = QDoubleSpinBox()
        self.ssl_spd2.setRange(-9999, 9999); self.ssl_spd2.setDecimals(2)
        self.ssl_spd2.setSuffix("  deg/s")
        gs.addWidget(self.ssl_spd2, 0, 3)

        btn_ssl_spd = QPushButton("Send SSL Speed")
        btn_ssl_spd.setProperty("class", "accent")
        btn_ssl_spd.clicked.connect(self._on_ssl_speed)
        gs.addWidget(btn_ssl_spd, 1, 3)
        outer.addWidget(grp_spd)

        outer.addStretch()
        return page

    # =====================================================================
    # Configuration tab
    # =====================================================================
    def _build_config_tab(self) -> QWidget:
        page = QWidget()
        outer = QVBoxLayout(page)
        outer.setSpacing(10)

        self.cfg_widgets: list[dict] = []
        for idx in range(2):
            axis_num = idx + 1
            grp = QGroupBox(f"Axis {axis_num} Configuration")
            grid = QGridLayout(grp)
            grid.setSpacing(8)
            w: dict = {}

            grid.addWidget(QLabel("Max Position:"), 0, 0)
            sp_max_pos = QDoubleSpinBox()
            sp_max_pos.setRange(0, 99999); sp_max_pos.setDecimals(2); sp_max_pos.setValue(400)
            sp_max_pos.setSuffix("  deg")
            grid.addWidget(sp_max_pos, 0, 1)
            w["max_pos"] = sp_max_pos

            grid.addWidget(QLabel("Max Speed:"), 1, 0)
            sp_max_spd = QDoubleSpinBox()
            sp_max_spd.setRange(0, 99999); sp_max_spd.setDecimals(2); sp_max_spd.setValue(20)
            sp_max_spd.setSuffix("  deg/s")
            grid.addWidget(sp_max_spd, 1, 1)
            w["max_spd"] = sp_max_spd

            grid.addWidget(QLabel("Max Acceleration:"), 2, 0)
            sp_max_acc = QDoubleSpinBox()
            sp_max_acc.setRange(0, 99999); sp_max_acc.setDecimals(2); sp_max_acc.setValue(120)
            sp_max_acc.setSuffix("  deg/s²")
            grid.addWidget(sp_max_acc, 2, 1)
            w["max_acc"] = sp_max_acc

            btn_apply = QPushButton("Apply Configuration")
            btn_apply.setProperty("class", "accent")
            btn_apply.clicked.connect(partial(self._apply_config, idx))
            grid.addWidget(btn_apply, 3, 0, 1, 2)
            outer.addWidget(grp)
            self.cfg_widgets.append(w)

        outer.addStretch()
        return page

    # =====================================================================
    # Combined Gimbal & Map tab
    # =====================================================================
    def _build_combined_ops_tab(self) -> QWidget:
        page = QWidget()
        outer = QHBoxLayout(page)
        outer.setContentsMargins(6, 6, 6, 6)
        outer.setSpacing(10)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        outer.addWidget(splitter)

        left_tabs = QTabWidget()
        left_tabs.setDocumentMode(True)
        left_tabs.setMinimumWidth(400)
        left_tabs.addTab(self._build_motor_controls_widget(), "Motors")
        left_tabs.addTab(self._build_map_sidebar_scroll(), "Map / Scan")
        splitter.addWidget(left_tabs)

        map_widget = self._build_map_view_widget()
        splitter.addWidget(map_widget)
        splitter.setSizes([440, 1000])

        self._wire_map_controls()
        self._refresh_preset_labels()
        self._update_map_connectivity_status()
        if self.map_view is not None:
            self._reload_map_engine()

        return page

    def _build_map_sidebar_scroll(self) -> QScrollArea:
        controls_scroll = QScrollArea()
        controls_scroll.setWidgetResizable(True)
        controls_scroll.setFrameShape(QFrame.NoFrame)
        controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        controls_layout.setContentsMargins(4, 4, 4, 4)
        controls_layout.setSpacing(10)
        controls_scroll.setWidget(controls)
        self._populate_map_sidebar(controls_layout)
        return controls_scroll

    def _populate_map_sidebar(self, controls_layout: QVBoxLayout) -> None:

        # --- Engine, layer, and connectivity status ---
        grp_engine = QGroupBox("Map Engine / Connectivity")
        eng = QGridLayout(grp_engine)
        eng.setSpacing(8)
        eng.setColumnStretch(1, 1)

        eng.addWidget(QLabel("Engine:"), 0, 0)
        self.map_engine = QComboBox()
        self.map_engine.addItem("Leaflet 2D", "leaflet")
        self.map_engine.addItem("MapLibre GL", "mapbox")
        self.map_engine.addItem("CesiumJS 3D", "cesium")
        eng.addWidget(self.map_engine, 0, 1)

        eng.addWidget(QLabel("Layer:"), 1, 0)
        self.map_layer = QComboBox()
        self.map_layer.addItems(["Terrain", "Satellite"])
        eng.addWidget(self.map_layer, 1, 1)

        eng.addWidget(QLabel("Zoom:"), 2, 0)
        self.map_zoom = QSpinBox()
        self.map_zoom.setRange(2, 20)
        self.map_zoom.setValue(12)
        eng.addWidget(self.map_zoom, 2, 1)

        self.map_follow = QCheckBox("Follow platform / camera focus")
        self.map_follow.setChecked(True)
        eng.addWidget(self.map_follow, 3, 0, 1, 2)

        self.map_online_indicator = StatusIndicator(size=12)
        self.lbl_map_online = QLabel("Map engine loading...")
        eng.addWidget(self.map_online_indicator, 4, 0)
        eng.addWidget(self.lbl_map_online, 4, 1)

        self.map_live_indicator = StatusIndicator(size=12)
        self.lbl_map_live = QLabel("Telemetry idle")
        eng.addWidget(self.map_live_indicator, 5, 0)
        eng.addWidget(self.lbl_map_live, 5, 1)

        self.btn_reload_map = QPushButton("Reload Engine")
        eng.addWidget(self.btn_reload_map, 6, 0, 1, 2)
        controls_layout.addWidget(grp_engine)

        # --- Map geometry legend ---
        grp_legend = QGroupBox("Map Geometry Legend")
        legend_layout = QVBoxLayout(grp_legend)
        legend_layout.setSpacing(4)
        self.map_show_legend = QCheckBox("Show legend on map")
        self.map_show_legend.setChecked(True)
        self.map_show_fill = QCheckBox("FOV volume fill (3D)")
        self.map_show_fill.setChecked(True)
        legend_layout.addWidget(self.map_show_legend)
        legend_layout.addWidget(self.map_show_fill)

        def _legend_row(color: str, text: str) -> QWidget:
            row = QWidget()
            h = QHBoxLayout(row)
            h.setContentsMargins(0, 0, 0, 0)
            swatch = QLabel("   ")
            swatch.setFixedWidth(22)
            swatch.setStyleSheet(
                f"background:{color}; border:1px solid #cdd6f4; border-radius:2px;"
            )
            h.addWidget(swatch)
            lbl = QLabel(text)
            lbl.setWordWrap(True)
            lbl.setProperty("class", "unit")
            h.addWidget(lbl, 1)
            return row

        legend_layout.addWidget(_legend_row("#89b4fa", "Platform / gimbal origin (E/N/U axes)"))
        legend_layout.addWidget(_legend_row("#ff8c00", "Live camera FOV (orange, dark near → dim far)"))
        legend_layout.addWidget(_legend_row("#ffcc00", "FOV edge rays + boresight (yellow/green)"))
        legend_layout.addWidget(_legend_row("#4da6ff", "Scan preview volume (continuous blue wedge)"))
        legend_layout.addWidget(_legend_row("#00ff88", "Active scan beam (current AZ/EL)"))
        legend_layout.addWidget(_legend_row("#f38ba8", "Boresight aim point"))
        self.map_show_legend.toggled.connect(self._on_map_legend_toggled)
        self.map_show_fill.toggled.connect(self._on_map_fill_toggled)
        controls_layout.addWidget(grp_legend)

        # --- LLA and heading with presets ---
        grp_geo = QGroupBox("Platform LLA / Heading Presets")
        geo = QGridLayout(grp_geo)
        geo.setSpacing(8)
        geo.setColumnStretch(1, 1)

        geo.addWidget(QLabel("Latitude:"), 0, 0)
        self.map_lat = QDoubleSpinBox()
        self.map_lat.setRange(-90.0, 90.0)
        self.map_lat.setDecimals(7)
        self.map_lat.setSingleStep(0.0001)
        self.map_lat.setValue(32.0853)
        geo.addWidget(self.map_lat, 0, 1)

        geo.addWidget(QLabel("Longitude:"), 1, 0)
        self.map_lon = QDoubleSpinBox()
        self.map_lon.setRange(-180.0, 180.0)
        self.map_lon.setDecimals(7)
        self.map_lon.setSingleStep(0.0001)
        self.map_lon.setValue(34.7818)
        geo.addWidget(self.map_lon, 1, 1)

        geo.addWidget(QLabel("Altitude (m):"), 2, 0)
        self.map_alt = QDoubleSpinBox()
        self.map_alt.setRange(0.0, 25000.0)
        self.map_alt.setDecimals(1)
        self.map_alt.setValue(120.0)
        geo.addWidget(self.map_alt, 2, 1)

        geo.addWidget(QLabel("Base Heading (deg):"), 3, 0)
        self.map_heading = QDoubleSpinBox()
        self.map_heading.setRange(0.0, 359.99)
        self.map_heading.setDecimals(2)
        self.map_heading.setValue(0.0)
        geo.addWidget(self.map_heading, 3, 1)

        self.btn_save_preset_1 = QPushButton("Save Preset 1")
        self.btn_load_preset_1 = QPushButton("Load Preset 1")
        self.btn_load_preset_1.setProperty("class", "accent")
        geo.addWidget(self.btn_save_preset_1, 4, 0)
        geo.addWidget(self.btn_load_preset_1, 4, 1)

        self.lbl_preset_1 = QLabel("Preset 1: not saved")
        self.lbl_preset_1.setProperty("class", "unit")
        geo.addWidget(self.lbl_preset_1, 5, 0, 1, 2)

        self.btn_save_preset_2 = QPushButton("Save Preset 2")
        self.btn_load_preset_2 = QPushButton("Load Preset 2")
        self.btn_load_preset_2.setProperty("class", "accent")
        geo.addWidget(self.btn_save_preset_2, 6, 0)
        geo.addWidget(self.btn_load_preset_2, 6, 1)

        self.lbl_preset_2 = QLabel("Preset 2: not saved")
        self.lbl_preset_2.setProperty("class", "unit")
        geo.addWidget(self.lbl_preset_2, 7, 0, 1, 2)
        controls_layout.addWidget(grp_geo)

        # --- FOV model ---
        grp_fov = QGroupBox("Sensor FOV / Footprint Model")
        fov = QGridLayout(grp_fov)
        fov.setSpacing(8)
        fov.setColumnStretch(1, 1)

        fov.addWidget(QLabel("Horizontal FOV (deg):"), 0, 0)
        self.map_hfov = QDoubleSpinBox()
        self.map_hfov.setRange(1.0, 180.0)
        self.map_hfov.setDecimals(1)
        self.map_hfov.setValue(24.0)
        fov.addWidget(self.map_hfov, 0, 1)

        fov.addWidget(QLabel("Vertical FOV (deg):"), 1, 0)
        self.map_vfov = QDoubleSpinBox()
        self.map_vfov.setRange(1.0, 120.0)
        self.map_vfov.setDecimals(1)
        self.map_vfov.setValue(14.0)
        fov.addWidget(self.map_vfov, 1, 1)

        fov.addWidget(QLabel("Look Distance (m):"), 2, 0)
        self.map_range = QDoubleSpinBox()
        self.map_range.setRange(10.0, 60000.0)
        self.map_range.setDecimals(1)
        self.map_range.setValue(1200.0)
        fov.addWidget(self.map_range, 2, 1)

        self.map_use_pitch_for_range = QCheckBox("Use altitude + pitch for realistic near/far footprint")
        self.map_use_pitch_for_range.setChecked(True)
        fov.addWidget(self.map_use_pitch_for_range, 3, 0, 1, 2)

        self.lbl_map_footprint = QLabel("Footprint: near --- m | center --- m | far --- m")
        self.lbl_map_footprint.setProperty("class", "unit")
        fov.addWidget(self.lbl_map_footprint, 4, 0, 1, 2)
        controls_layout.addWidget(grp_fov)

        # --- Gimbal coupling ---
        grp_sync = QGroupBox("Gimbal Coupling (Fixed Axis Mapping)")
        sync = QGridLayout(grp_sync)
        sync.setSpacing(8)
        sync.setColumnStretch(1, 1)

        self.lbl_axis_mapping = QLabel("Axis 1 = Yaw / Heading, Axis 2 = Tilt-Pitch")
        self.lbl_axis_mapping.setProperty("class", "heading")
        sync.addWidget(self.lbl_axis_mapping, 0, 0, 1, 2)

        self.map_use_gimbal_heading = QCheckBox("Apply yaw to heading")
        self.map_use_gimbal_heading.setChecked(True)
        sync.addWidget(self.map_use_gimbal_heading, 1, 0, 1, 2)

        sync.addWidget(QLabel("Live Yaw (Axis 1, deg):"), 2, 0)
        self.lbl_map_yaw = QLabel("---")
        self.lbl_map_yaw.setProperty("class", "map-value")
        sync.addWidget(self.lbl_map_yaw, 2, 1)

        sync.addWidget(QLabel("Live Pitch / Tilt (Axis 2, deg):"), 3, 0)
        self.lbl_map_pitch = QLabel("---")
        self.lbl_map_pitch.setProperty("class", "map-value")
        sync.addWidget(self.lbl_map_pitch, 3, 1)

        sync.addWidget(QLabel("Live Heading (deg):"), 4, 0)
        self.lbl_map_heading = QLabel("---")
        self.lbl_map_heading.setProperty("class", "map-value")
        sync.addWidget(self.lbl_map_heading, 4, 1)

        self.lbl_map_sync_state = QLabel("Not synced")
        self.lbl_map_sync_state.setProperty("class", "unit")
        sync.addWidget(self.lbl_map_sync_state, 5, 0, 1, 2)

        self.btn_map_sync_live = QPushButton("Sync Live")
        self.btn_map_sync_live.setProperty("class", "accent")
        self.btn_map_sync_live.clicked.connect(self._on_map_sync_live)
        sync.addWidget(self.btn_map_sync_live, 6, 0, 1, 2)

        self.btn_map_read_axes = QPushButton("Read Axes Now")
        self.btn_map_read_axes.clicked.connect(self._poll_positions)
        sync.addWidget(self.btn_map_read_axes, 7, 0, 1, 2)
        controls_layout.addWidget(grp_sync)

        # --- Display controls ---
        grp_map = QGroupBox("Display Actions")
        map_grid = QGridLayout(grp_map)
        map_grid.setSpacing(8)
        map_grid.setColumnStretch(1, 1)

        self.btn_map_update = QPushButton("Update Overlay")
        self.btn_map_update.setProperty("class", "accent")
        map_grid.addWidget(self.btn_map_update, 0, 0, 1, 2)

        self.btn_map_center = QPushButton("Center on Platform")
        map_grid.addWidget(self.btn_map_center, 1, 0, 1, 2)
        controls_layout.addWidget(grp_map)

        # --- 3D camera control (Cesium) ---
        grp_cam = QGroupBox("3D Camera / Sensor POV")
        grp_cam.setStyleSheet("QGroupBox{font-weight:bold;color:#89b4fa;}")
        cam = QGridLayout(grp_cam)
        cam.setSpacing(6)
        cam.setColumnStretch(1, 1)

        cam.addWidget(QLabel("View mode:"), 0, 0)
        self.cesium_view_mode = QComboBox()
        self.cesium_view_mode.addItem("Globe overview", "globe")
        self.cesium_view_mode.addItem("Orbit platform", "orbit")
        self.cesium_view_mode.addItem("Sensor POV (camera)", "sensor")
        cam.addWidget(self.cesium_view_mode, 0, 1)

        cam.addWidget(QLabel("Orbit bearing (deg):"), 1, 0)
        self.cesium_orbit_bearing = QDoubleSpinBox()
        self.cesium_orbit_bearing.setRange(-180.0, 360.0)
        self.cesium_orbit_bearing.setDecimals(1)
        self.cesium_orbit_bearing.setValue(45.0)
        cam.addWidget(self.cesium_orbit_bearing, 1, 1)

        cam.addWidget(QLabel("Orbit pitch (deg):"), 2, 0)
        self.cesium_orbit_pitch = QDoubleSpinBox()
        self.cesium_orbit_pitch.setRange(-89.0, 0.0)
        self.cesium_orbit_pitch.setDecimals(1)
        self.cesium_orbit_pitch.setValue(-35.0)
        cam.addWidget(self.cesium_orbit_pitch, 2, 1)

        cam.addWidget(QLabel("Orbit range (m):"), 3, 0)
        self.cesium_orbit_range = QDoubleSpinBox()
        self.cesium_orbit_range.setRange(50.0, 80000.0)
        self.cesium_orbit_range.setDecimals(0)
        self.cesium_orbit_range.setValue(2500.0)
        cam.addWidget(self.cesium_orbit_range, 3, 1)

        cam_btns = QHBoxLayout()
        self.btn_cam_left = QPushButton("\u2190 Yaw")
        self.btn_cam_right = QPushButton("Yaw \u2192")
        self.btn_cam_up = QPushButton("\u2191 Pitch")
        self.btn_cam_down = QPushButton("Pitch \u2193")
        self.btn_cam_zoom_in = QPushButton("Zoom +")
        self.btn_cam_zoom_out = QPushButton("Zoom -")
        for b in (
            self.btn_cam_left, self.btn_cam_right, self.btn_cam_up,
            self.btn_cam_down, self.btn_cam_zoom_in, self.btn_cam_zoom_out,
        ):
            b.setMaximumWidth(72)
            cam_btns.addWidget(b)
        cam.addLayout(cam_btns, 4, 0, 1, 2)

        self.btn_apply_cam = QPushButton("Apply Camera View")
        self.btn_apply_cam.setProperty("class", "accent")
        cam.addWidget(self.btn_apply_cam, 5, 0, 1, 2)

        self.cesium_view_mode.currentIndexChanged.connect(self._on_cesium_view_mode_changed)
        self.cesium_orbit_bearing.valueChanged.connect(self._apply_cesium_camera)
        self.cesium_orbit_pitch.valueChanged.connect(self._apply_cesium_camera)
        self.cesium_orbit_range.valueChanged.connect(self._apply_cesium_camera)
        self.btn_apply_cam.clicked.connect(self._apply_cesium_camera)
        self.btn_cam_left.clicked.connect(lambda: self._nudge_cesium_camera(-10, 0, 0))
        self.btn_cam_right.clicked.connect(lambda: self._nudge_cesium_camera(10, 0, 0))
        self.btn_cam_up.clicked.connect(lambda: self._nudge_cesium_camera(0, 5, 0))
        self.btn_cam_down.clicked.connect(lambda: self._nudge_cesium_camera(0, -5, 0))
        self.btn_cam_zoom_in.clicked.connect(lambda: self._nudge_cesium_camera(0, 0, -400))
        self.btn_cam_zoom_out.clicked.connect(lambda: self._nudge_cesium_camera(0, 0, 400))
        controls_layout.addWidget(grp_cam)

        # --- Scan pattern ---
        grp_scan = QGroupBox("Scan Pattern")
        grp_scan.setStyleSheet("QGroupBox{font-weight:bold;color:#fab387;}")
        scan = QGridLayout(grp_scan)
        scan.setSpacing(8)
        scan.setColumnStretch(1, 1)

        scan.addWidget(QLabel("Mode:"), 0, 0)
        self.scan_mode = QComboBox()
        self.scan_mode.addItem("Map preview only", "demo")
        self.scan_mode.addItem("Gimbal hardware", "hardware")
        self.scan_mode.currentIndexChanged.connect(self._on_scan_mode_changed)
        scan.addWidget(self.scan_mode, 0, 1)

        scan.addWidget(QLabel("Azimuth Start (deg):"), 1, 0)
        self.scan_az_start = QDoubleSpinBox()
        self.scan_az_start.setRange(-180.0, 360.0)
        self.scan_az_start.setDecimals(1)
        self.scan_az_start.setValue(-30.0)
        scan.addWidget(self.scan_az_start, 1, 1)

        scan.addWidget(QLabel("Azimuth End (deg):"), 2, 0)
        self.scan_az_end = QDoubleSpinBox()
        self.scan_az_end.setRange(-180.0, 360.0)
        self.scan_az_end.setDecimals(1)
        self.scan_az_end.setValue(30.0)
        scan.addWidget(self.scan_az_end, 2, 1)

        scan.addWidget(QLabel("Elevation Start (deg):"), 3, 0)
        self.scan_el_start = QDoubleSpinBox()
        self.scan_el_start.setRange(-90.0, 90.0)
        self.scan_el_start.setDecimals(1)
        self.scan_el_start.setValue(-5.0)
        scan.addWidget(self.scan_el_start, 3, 1)

        scan.addWidget(QLabel("Elevation End (deg):"), 4, 0)
        self.scan_el_end = QDoubleSpinBox()
        self.scan_el_end.setRange(-90.0, 90.0)
        self.scan_el_end.setDecimals(1)
        self.scan_el_end.setValue(-25.0)
        scan.addWidget(self.scan_el_end, 4, 1)

        scan.addWidget(QLabel("Scan Speed (deg/s):"), 5, 0)
        self.scan_speed = QDoubleSpinBox()
        self.scan_speed.setRange(0.5, 200.0)
        self.scan_speed.setDecimals(1)
        self.scan_speed.setValue(15.0)
        scan.addWidget(self.scan_speed, 5, 1)

        self.btn_calc_scan = QPushButton("CALCULATE")
        self.btn_calc_scan.setProperty("class", "accent")
        self.btn_start_scan = QPushButton("START SCAN")
        self.btn_start_scan.setStyleSheet(
            "QPushButton{background:#a6e3a1;color:#111;font-weight:bold;}"
            "QPushButton:hover{background:#74c78b;}"
        )
        self.btn_stop_scan = QPushButton("ABORT")
        self.btn_stop_scan.setStyleSheet(
            "QPushButton{background:#f38ba8;color:#111;font-weight:bold;}"
            "QPushButton:hover{background:#e06080;}"
        )
        self.btn_stop_scan.setEnabled(False)

        self.scan_loop = QCheckBox("Loop scan continuously")
        self.scan_loop.setChecked(False)
        self.scan_loop.setToolTip("Map preview only — hardware snake scan is controller-managed.")
        scan.addWidget(self.scan_loop, 6, 0, 1, 2)

        self.btn_preview_scan = QPushButton("PREVIEW SCAN AREA")
        self.btn_preview_scan.setStyleSheet(
            "QPushButton{background:#89b4fa;color:#111;font-weight:bold;}"
        )
        scan.addWidget(self.btn_preview_scan, 7, 0, 1, 2)

        scan.addWidget(self.btn_calc_scan, 8, 0)
        scan.addWidget(self.btn_start_scan, 8, 1)
        scan.addWidget(self.btn_stop_scan, 9, 0, 1, 2)

        self.lbl_scan_info = QLabel("AZ range: --- | EL range: --- | Passes: ---")
        self.lbl_scan_info.setProperty("class", "unit")
        self.lbl_scan_info.setWordWrap(True)
        scan.addWidget(self.lbl_scan_info, 10, 0, 1, 2)

        self.lbl_scan_time = QLabel("Est. time: --- | Area: ---")
        self.lbl_scan_time.setProperty("class", "map-value")
        self.lbl_scan_time.setWordWrap(True)
        scan.addWidget(self.lbl_scan_time, 11, 0, 1, 2)

        self.scan_progress = QProgressBar()
        self.scan_progress.setRange(0, 1000)
        self.scan_progress.setValue(0)
        self.scan_progress.setTextVisible(True)
        self.scan_progress.setFormat("IDLE")
        self.scan_progress.setStyleSheet(
            "QProgressBar{background:#1e1e2e;border:1px solid #585b70;border-radius:4px;text-align:center;color:#cdd6f4;}"
            "QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
            "stop:0 #a6e3a1, stop:0.5 #89b4fa, stop:1 #cba6f7);border-radius:3px;}"
        )
        scan.addWidget(self.scan_progress, 12, 0, 1, 2)

        self.btn_calc_scan.clicked.connect(self._calc_scan)
        self.btn_preview_scan.clicked.connect(self._preview_scan_area)
        self.btn_start_scan.clicked.connect(self._start_scan)
        self.btn_stop_scan.clicked.connect(self._stop_scan)
        controls_layout.addWidget(grp_scan)

        # --- Controller integration ---
        grp_api = QGroupBox("Controller API")
        api = QVBoxLayout(grp_api)
        self.btn_send_fov_api = QPushButton("Send FOV To Controller")
        self.btn_send_fov_api.setProperty("class", "warning")
        api.addWidget(self.btn_send_fov_api)
        controls_layout.addWidget(grp_api)

        controls_layout.addStretch(1)

    def _build_map_view_widget(self) -> QWidget:
        if WEBENGINE_AVAILABLE:
            self.map_view = QWebEngineView()
            settings = self.map_view.page().settings()
            settings.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
            settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
            self.map_view.loadFinished.connect(self._on_map_loaded)
            return self.map_view

        self.map_view = None
        missing = QLabel(
            "QWebEngine is not available.\n"
            "Install PyQtWebEngine to enable the tactical map view."
        )
        missing.setAlignment(Qt.AlignCenter)
        missing.setWordWrap(True)
        return missing

    def _wire_map_controls(self) -> None:
        for field in (
            self.map_lat, self.map_lon, self.map_alt, self.map_heading,
            self.map_hfov, self.map_vfov, self.map_range,
        ):
            field.valueChanged.connect(self._schedule_map_overlay_refresh)

        self.map_follow.toggled.connect(self._schedule_map_overlay_refresh)
        self.map_use_gimbal_heading.toggled.connect(self._schedule_map_overlay_refresh)
        self.map_use_pitch_for_range.toggled.connect(self._schedule_map_overlay_refresh)
        self.map_layer.currentTextChanged.connect(self._on_map_layer_changed)
        self.map_zoom.valueChanged.connect(self._on_map_zoom_changed)
        self.map_engine.currentIndexChanged.connect(self._on_map_engine_changed)

        self.btn_map_update.clicked.connect(
            lambda: self._refresh_map_overlay(pan_map=self.map_follow.isChecked())
        )
        self.btn_map_center.clicked.connect(
            lambda: self._refresh_map_overlay(pan_map=True)
        )
        self.btn_send_fov_api.clicked.connect(self._send_map_fov_to_controller)
        self.btn_reload_map.clicked.connect(self._reload_map_engine)
        self.btn_save_preset_1.clicked.connect(lambda: self._save_preset("preset_1"))
        self.btn_load_preset_1.clicked.connect(lambda: self._load_preset("preset_1"))
        self.btn_save_preset_2.clicked.connect(lambda: self._save_preset("preset_2"))
        self.btn_load_preset_2.clicked.connect(lambda: self._load_preset("preset_2"))
        self._on_scan_mode_changed()

    def _build_map_html(self, engine: str) -> str:
        if engine == "mapbox":
            return self._mapbox_html()
        if engine == "cesium":
            return self._cesium_html()
        return self._leaflet_html()

    @staticmethod
    def _leaflet_html() -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1.0" />
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
  <style>
    html, body, #map {
      margin: 0; height: 100%; width: 100%;
      background: #0d1117; color: #d7e3fc;
      font-family: "Courier New", monospace;
    }
    .leaflet-control-attribution {
      background: rgba(13,17,23,0.72) !important;
      color: #b8c4db !important;
      border-top: 1px solid rgba(137,180,250,0.35);
      font-family: "Courier New", monospace;
    }
    .leaflet-control-attribution a { color: #89b4fa !important; }
    .leaflet-tooltip {
      background: rgba(17,24,39,0.95); color: #d7e3fc;
      border: 1px solid #89b4fa; border-radius: 6px;
      font-weight: 600; font-family: "Courier New", monospace;
      text-transform: uppercase;
      box-shadow: 0 4px 16px rgba(0,0,0,0.35);
    }
    #offlineBanner {
      position: fixed; top: 8px; left: 50%; transform: translateX(-50%);
      background: rgba(85,107,47,0.92); color: #f9e2af;
      padding: 5px 14px; border-radius: 3px; z-index: 1000;
      font: bold 11px "Courier New", monospace;
      text-transform: uppercase; letter-spacing: 1px;
      display: none; border: 1px solid rgba(249,226,175,0.4);
    }
    #radarHud {
      position: fixed; top: 10px; right: 10px; z-index: 1100;
      background: rgba(13,17,23,0.85);
      border: 1px solid rgba(137,180,250,0.3);
      border-radius: 6px; padding: 8px; pointer-events: none;
    }
    #coordOverlay {
      position: fixed; bottom: 26px; left: 8px; z-index: 1100;
      background: rgba(13,17,23,0.75);
      border: 1px solid rgba(137,180,250,0.2);
      border-radius: 4px; padding: 4px 8px;
      font: 10px "Courier New", monospace;
      color: #a6e3a1; text-transform: uppercase;
      pointer-events: none;
    }
  </style>
</head>
<body>
  <div id="offlineBanner">OFFLINE - MAP TILES UNAVAILABLE</div>
  <div id="map"></div>
  <div id="radarHud"><canvas id="hudCanvas" width="180" height="220"></canvas></div>
  <div id="coordOverlay">LAT ---.------ LON ---.------</div>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    var status = {
      engine: "Leaflet 2D",
      ready: false,
      interactive: false,
      online: navigator.onLine
    };

    function setOfflineBanner(online) {
      document.getElementById("offlineBanner").style.display = online ? "none" : "block";
      status.online = !!online;
    }
    window.addEventListener("online", function() { setOfflineBanner(true); });
    window.addEventListener("offline", function() { setOfflineBanner(false); });
    setOfflineBanner(navigator.onLine);

    var terrainLayer = L.tileLayer(
      "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
      { maxZoom: 17, attribution: "&copy; OpenTopoMap, OpenStreetMap contributors" }
    );
    var satelliteLayer = L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      { maxZoom: 19, attribution: "Tiles &copy; Esri" }
    );

    var map = L.map("map", {
      zoomControl: true,
      preferCanvas: true,
      layers: [terrainLayer]
    }).setView([32.0853, 34.7818], 12);

    var platformMarker = L.circleMarker([32.0853, 34.7818], {
      radius: 8, color: "#89b4fa", weight: 2,
      fillColor: "#89b4fa", fillOpacity: 0.95
    }).addTo(map);

    var headingLine = L.polyline([[32.0853, 34.7818], [32.09, 34.79]], {
      color: "#a6e3a1", weight: 3, opacity: 0.95
    }).addTo(map);

    var targetMarker = L.circleMarker([32.09, 34.79], {
      radius: 6, color: "#f38ba8", weight: 2,
      fillColor: "#f38ba8", fillOpacity: 0.95
    }).addTo(map);

    var fovPolygon = L.polygon([], {
      color: "#f9e2af", weight: 1, opacity: 0.4,
      fillColor: "#f9e2af", fillOpacity: 0.0
    }).addTo(map);
    var nearArc = L.polyline([], {
      color: "#fab387", weight: 2, opacity: 0.9, dashArray: "5,6"
    }).addTo(map);
    var farArc = L.polyline([], {
      color: "#89b4fa", weight: 2, opacity: 0.9, dashArray: "4,7"
    }).addTo(map);
    var centerArc = L.polyline([], {
      color: "#f9e2af", weight: 1.5, opacity: 0.6, dashArray: "3,5"
    }).addTo(map);

    var activeLayer = "terrain";
    var platformState = {
      lat: 32.0853, lon: 34.7818,
      heading: 0.0, hfov: 24.0,
      pitch: 0.0, vfov: 14.0,
      nearDist: 100, centerDist: 600, farDist: 1200
    };

    function destinationPoint(lat, lon, bearingDeg, distanceMeters) {
      var R = 6378137.0;
      var br = bearingDeg * Math.PI / 180.0;
      var la = lat * Math.PI / 180.0;
      var lo = lon * Math.PI / 180.0;
      var d = distanceMeters / R;
      var sla = Math.sin(la), cla = Math.cos(la);
      var sd = Math.sin(d), cd = Math.cos(d);
      var la2 = Math.asin(sla * cd + cla * sd * Math.cos(br));
      var lo2 = lo + Math.atan2(Math.sin(br) * sd * cla, cd - sla * Math.sin(la2));
      return [la2 * 180.0 / Math.PI, lo2 * 180.0 / Math.PI];
    }

    function buildArc(lat, lon, headingDeg, hfovDeg, distanceMeters) {
      var half = Math.max(0.5, hfovDeg / 2.0);
      var start = headingDeg - half;
      var end = headingDeg + half;
      var step = Math.max(1.0, hfovDeg / 16.0);
      var points = [];
      for (var b = start; b <= end + 0.001; b += step) {
        points.push(destinationPoint(lat, lon, b, distanceMeters));
      }
      return points;
    }

    function buildFovPolygon(lat, lon, headingDeg, hfovDeg, nearDistance, farDistance) {
      var outer = buildArc(lat, lon, headingDeg, hfovDeg, farDistance);
      var inner = buildArc(lat, lon, headingDeg, hfovDeg, nearDistance).reverse();
      return outer.concat(inner);
    }

    /* ---- Canvas gradient FOV overlay ---- */
    var fovCanvas = document.createElement("canvas");
    fovCanvas.style.cssText = "position:absolute;top:0;left:0;pointer-events:none;z-index:450;";
    map.getPane("overlayPane").appendChild(fovCanvas);

    function redrawFovGradient() {
      var sz = map.getSize();
      fovCanvas.width = sz.x;
      fovCanvas.height = sz.y;
      var ctx = fovCanvas.getContext("2d");
      ctx.clearRect(0, 0, sz.x, sz.y);
      var s = platformState;
      var polyLL = buildFovPolygon(s.lat, s.lon, s.heading, s.hfov, s.nearDist, s.farDist);
      if (!polyLL.length) return;
      var org = map.latLngToContainerPoint([s.lat, s.lon]);
      var polyPx = polyLL.map(function(p) { return map.latLngToContainerPoint(p); });
      var maxR = 10;
      polyPx.forEach(function(p) {
        var d = Math.hypot(p.x - org.x, p.y - org.y);
        if (d > maxR) maxR = d;
      });
      var grad = ctx.createRadialGradient(org.x, org.y, 0, org.x, org.y, maxR);
      grad.addColorStop(0,    "rgba(249,226,175,0.68)");
      grad.addColorStop(0.22, "rgba(249,226,175,0.48)");
      grad.addColorStop(0.50, "rgba(249,226,175,0.26)");
      grad.addColorStop(0.78, "rgba(249,226,175,0.12)");
      grad.addColorStop(1,    "rgba(249,226,175,0.03)");
      ctx.beginPath();
      polyPx.forEach(function(p, i) { i ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y); });
      ctx.closePath();
      ctx.fillStyle = grad;
      ctx.fill();
      ctx.lineWidth = 2.5;
      ctx.strokeStyle = "rgba(250,179,135,0.95)";
      ctx.stroke();

      /* scan lines inside the FOV cone */
      ctx.strokeStyle = "rgba(249,226,175,0.10)";
      ctx.lineWidth = 1;
      for (var i = 0; i <= 4; i++) {
        var a = s.heading - s.hfov / 2 + s.hfov * i / 4;
        var fp = map.latLngToContainerPoint(destinationPoint(s.lat, s.lon, a, s.farDist));
        ctx.beginPath();
        ctx.moveTo(org.x, org.y);
        ctx.lineTo(fp.x, fp.y);
        ctx.stroke();
      }

      /* concentric range rings with distance labels */
      var rings = [
        { d: s.nearDist,   lbl: "NEAR", c: "rgba(250,179,135,0.70)" },
        { d: s.centerDist, lbl: "CTR",  c: "rgba(249,226,175,0.60)" },
        { d: s.farDist,    lbl: "FAR",  c: "rgba(137,180,250,0.70)" }
      ];
      rings.forEach(function(ring) {
        var arc = buildArc(s.lat, s.lon, s.heading, s.hfov, ring.d)
          .map(function(p) { return map.latLngToContainerPoint(p); });
        if (!arc.length) return;
        ctx.beginPath();
        arc.forEach(function(p, idx) { idx ? ctx.lineTo(p.x, p.y) : ctx.moveTo(p.x, p.y); });
        ctx.strokeStyle = ring.c;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        var mid = arc[Math.floor(arc.length / 2)];
        ctx.font = "10px Courier New";
        ctx.fillStyle = ring.c;
        ctx.fillText(ring.lbl + " " + Math.round(ring.d) + "m", mid.x + 4, mid.y - 4);
      });
    }
    map.on("move zoom viewreset resize", redrawFovGradient);

    /* ---- Radar HUD overlay ---- */
    function drawRadarHud() {
      var cv = document.getElementById("hudCanvas");
      var ctx = cv.getContext("2d");
      var W = 180, H = 220;
      ctx.clearRect(0, 0, W, H);
      var s = platformState;
      var cx = 80, cy = 85, r = 68;

      /* compass ring */
      ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI);
      ctx.strokeStyle = "rgba(137,180,250,0.35)"; ctx.lineWidth = 1; ctx.stroke();
      ctx.beginPath(); ctx.arc(cx, cy, r * 0.5, 0, 2 * Math.PI);
      ctx.strokeStyle = "rgba(137,180,250,0.15)"; ctx.stroke();

      /* degree ticks every 30 deg */
      for (var d = 0; d < 360; d += 30) {
        var rad = (d - 90) * Math.PI / 180;
        var r1 = r - 6, r2 = r;
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(rad) * r1, cy + Math.sin(rad) * r1);
        ctx.lineTo(cx + Math.cos(rad) * r2, cy + Math.sin(rad) * r2);
        ctx.strokeStyle = d % 90 === 0 ? "rgba(205,214,244,0.7)" : "rgba(205,214,244,0.35)";
        ctx.lineWidth = d % 90 === 0 ? 1.5 : 1;
        ctx.stroke();
      }

      /* cardinal labels */
      ctx.font = "bold 10px Courier New";
      ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillStyle = "#cdd6f4";
      ctx.fillText("N", cx, cy - r - 8);
      ctx.fillText("S", cx, cy + r + 8);
      ctx.fillText("E", cx + r + 10, cy);
      ctx.fillText("W", cx - r - 10, cy);

      /* HFoV arc wedge */
      var hRad = (s.heading - 90) * Math.PI / 180;
      var halfF = s.hfov / 2 * Math.PI / 180;
      ctx.beginPath(); ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, r - 10, hRad - halfF, hRad + halfF);
      ctx.closePath();
      ctx.fillStyle = "rgba(249,226,175,0.25)"; ctx.fill();
      ctx.strokeStyle = "rgba(249,226,175,0.6)"; ctx.lineWidth = 1; ctx.stroke();

      /* heading line with glow */
      var hx = cx + Math.cos(hRad) * (r + 2);
      var hy = cy + Math.sin(hRad) * (r + 2);
      ctx.save();
      ctx.shadowColor = "#a6e3a1"; ctx.shadowBlur = 6;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy);
      ctx.strokeStyle = "#a6e3a1"; ctx.lineWidth = 2; ctx.stroke();
      ctx.restore();

      /* heading readout */
      ctx.font = "10px Courier New"; ctx.fillStyle = "#a6e3a1";
      ctx.textAlign = "center";
      ctx.fillText("HDG " + s.heading.toFixed(1) + "\u00b0", cx, cy + r + 20);

      /* elevation bar (right side) */
      var ex = 162, ey = 20, eH = 120, eW = 12;
      ctx.strokeStyle = "rgba(137,180,250,0.3)"; ctx.lineWidth = 1;
      ctx.strokeRect(ex, ey, eW, eH);
      /* horizon line */
      var hzY = ey + eH / 2;
      ctx.beginPath(); ctx.moveTo(ex - 2, hzY); ctx.lineTo(ex + eW + 2, hzY);
      ctx.strokeStyle = "rgba(205,214,244,0.25)"; ctx.stroke();
      /* pitch marker:  -90 bottom, 0 center, +90 top */
      var pNorm = (-s.pitch + 90) / 180;
      var pY = ey + pNorm * eH;
      var vN = s.vfov / 180 * eH;
      ctx.fillStyle = "rgba(249,226,175,0.2)";
      ctx.fillRect(ex, pY - vN / 2, eW, vN);
      ctx.beginPath(); ctx.moveTo(ex - 2, pY); ctx.lineTo(ex + eW + 2, pY);
      ctx.strokeStyle = "#f9e2af"; ctx.lineWidth = 2; ctx.stroke();
      ctx.font = "9px Courier New"; ctx.fillStyle = "#f9e2af"; ctx.textAlign = "center";
      ctx.fillText("EL", ex + eW / 2, ey - 6);
      ctx.fillText(s.pitch.toFixed(1) + "\u00b0", ex + eW / 2, ey + eH + 12);
    }

    /* ---- Coordinate overlay ---- */
    function updateCoordOverlay() {
      document.getElementById("coordOverlay").textContent =
        "LAT " + platformState.lat.toFixed(6) + " LON " + platformState.lon.toFixed(6);
    }

    /* ---- Map API (preserved call signatures) ---- */
    window.setMapLayer = function(mode) {
      var normalized = (mode || "").toLowerCase();
      if (normalized === activeLayer) return;
      if (activeLayer === "satellite") {
        map.removeLayer(satelliteLayer);
      } else {
        map.removeLayer(terrainLayer);
      }
      if (normalized === "satellite") {
        satelliteLayer.addTo(map);
        activeLayer = "satellite";
      } else {
        terrainLayer.addTo(map);
        activeLayer = "terrain";
      }
    };

    window.setMapZoom = function(level) {
      if (Number.isFinite(level)) {
        map.setZoom(level);
      }
    };

    window.centerPlatform = function() {
      map.panTo([platformState.lat, platformState.lon], { animate: true, duration: 0.35 });
    };

    function resolveBoresightHeading(payload) {
      if (payload && payload.boresightHeading != null) {
        return Number(payload.boresightHeading);
      }
      if (payload && payload.heading != null) {
        return Number(payload.heading);
      }
      return Number(platformState.heading);
    }

    function resolveBoresightDistance(payload) {
      if (payload && payload.boresightDistance != null) {
        return Math.max(10.0, Number(payload.boresightDistance));
      }
      if (payload && payload.centerDistance != null) {
        return Math.max(10.0, Number(payload.centerDistance));
      }
      return Math.max(10.0, Number(platformState.centerDist));
    }

    function applyLookGeometry(lat, lon, hdg, hfov, nearD, centerD, farD) {
      var center = [lat, lon];
      var aim = destinationPoint(lat, lon, hdg, centerD);
      platformMarker.setLatLng(center);
      headingLine.setLatLngs([center, aim]);
      targetMarker.setLatLng(aim);
      var poly = buildFovPolygon(lat, lon, hdg, hfov, nearD, farD);
      fovPolygon.setLatLngs(poly);
      nearArc.setLatLngs(buildArc(lat, lon, hdg, hfov, nearD));
      centerArc.setLatLngs(buildArc(lat, lon, hdg, hfov, centerD));
      farArc.setLatLngs(buildArc(lat, lon, hdg, hfov, farD));
    }

    window.updatePlatform = function(payload) {
      if (!payload) return;

      var lat = Number(payload.lat);
      var lon = Number(payload.lon);
      var altM = Math.max(1.0, Number(payload.alt || 120));
      var pitch = Number(payload.pitch || 0);
      var hdg = resolveBoresightHeading(payload);
      var hfov = Number(payload.hfov);
      var vfov = Number(payload.vfov || 14);
      var nearDistance = Math.max(10.0, Number(payload.nearDistance || 100.0));
      var centerDistance = resolveBoresightDistance(payload);
      var farDistance = Math.max(nearDistance + 5.0, Number(payload.farDistance || 200.0));

      platformState.lat = lat;
      platformState.lon = lon;
      platformState.heading = hdg;
      platformState.hfov = hfov;
      platformState.pitch = pitch;
      platformState.vfov = vfov;
      platformState.nearDist = nearDistance;
      platformState.centerDist = centerDistance;
      platformState.farDist = farDistance;

      platformMarker.bindTooltip(
        payload.platformLabel || "Gimbal platform (LLA origin)",
        { permanent: false, direction: "top" }
      );
      applyLookGeometry(lat, lon, hdg, hfov, nearDistance, centerDistance, farDistance);
      if (payload.targetLabel) {
        targetMarker.bindTooltip(payload.targetLabel, { permanent: false, direction: "right" });
      }
      if (payload.label) {
        platformMarker.bindTooltip(payload.label, { permanent: false, direction: "left" });
      }

      redrawFovGradient();
      drawRadarHud();
      updateCoordOverlay();

      if (payload.follow) {
        map.panTo([lat, lon], { animate: true, duration: 0.25 });
      }
    };

    map.on("move zoom resize", function() { redrawFovGradient(); });

    /* ---- Scan progress overlay ---- */
    var scanMarkers = [];
    var scanBeamMarker = null;

    window.updateScanProgress = function(payload) {
      if (!payload) return;
      var el = Number(payload.el);
      var hdg = resolveBoresightHeading(payload);
      var dist = resolveBoresightDistance(payload);
      var pt = destinationPoint(platformState.lat, platformState.lon, hdg, dist);
      if (scanBeamMarker) {
        scanBeamMarker.setLatLng(pt);
      } else {
        scanBeamMarker = L.circleMarker(pt, {
          radius: 5, color: "#f9e2af", weight: 2,
          fillColor: "#f9e2af", fillOpacity: 0.9
        }).addTo(map);
      }
      var trail = L.circleMarker(pt, {
        radius: 2, color: "#a6e3a1", weight: 0,
        fillColor: "#a6e3a1", fillOpacity: 0.6
      }).addTo(map);
      scanMarkers.push(trail);
    };

    window.clearScanOverlay = function() {
      scanMarkers.forEach(function(m) { map.removeLayer(m); });
      scanMarkers.length = 0;
      if (scanBeamMarker) { map.removeLayer(scanBeamMarker); scanBeamMarker = null; }
    };

    var scanPreviewLayer = null;
    window.hideScanAreaPreview = function() {
      if (scanPreviewLayer) { map.removeLayer(scanPreviewLayer); scanPreviewLayer = null; }
    };
    window.showScanAreaPreview = function(params) {
      if (!params) return;
      window.hideScanAreaPreview();
      var lat = Number(params.lat), lon = Number(params.lon), alt = Number(params.alt || 120);
      var base = Number(params.base_heading || 0);
      var az0 = Number(params.az_start), az1 = Number(params.az_end);
      var el0 = Number(params.el_start), el1 = Number(params.el_end);
      var azMid = base + (az0 + az1) / 2, azSpan = Math.abs(az1 - az0);
      var distTop = alt / Math.tan(Math.max(0.8, Math.abs(el0)) * Math.PI / 180);
      var distBot = alt / Math.tan(Math.max(0.8, Math.abs(el1)) * Math.PI / 180);
      var arcTop = buildArc(lat, lon, azMid, Math.max(azSpan, 10), distTop);
      var arcBot = buildArc(lat, lon, azMid, Math.max(azSpan, 10), distBot).reverse();
      scanPreviewLayer = L.polygon(arcTop.concat(arcBot), {
        color: "#89b4fa", weight: 3, dashArray: "8,6",
        fillColor: "#89b4fa", fillOpacity: 0.2
      }).addTo(map);
    };
    window.waitForMapReady = function() { return Promise.resolve(true); };
    window.setCameraView = function() {};

    status.ready = true;
    status.interactive = true;
    window.getMapStatus = function() {
      return {
        engine: status.engine,
        ready: status.ready,
        interactive: status.interactive,
        online: navigator.onLine
      };
    };
  </script>
</body>
</html>
"""

    @staticmethod
    def _mapbox_html() -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1.0" />
  <link href="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.css" rel="stylesheet" />
  <style>
    html, body, #map {
      margin: 0; height: 100%; width: 100%;
      background: #0d1117;
      font-family: "Courier New", monospace;
    }
    #offlineBanner {
      position: fixed; top: 8px; left: 50%; transform: translateX(-50%);
      background: rgba(85,107,47,0.92); color: #f9e2af;
      padding: 5px 14px; border-radius: 3px; z-index: 1000;
      font: bold 11px "Courier New", monospace;
      text-transform: uppercase; letter-spacing: 1px;
      display: none; border: 1px solid rgba(249,226,175,0.4);
    }
    #radarHud {
      position: fixed; top: 10px; right: 60px; z-index: 1100;
      background: rgba(13,17,23,0.85);
      border: 1px solid rgba(137,180,250,0.3);
      border-radius: 6px; padding: 8px; pointer-events: none;
    }
    #coordOverlay {
      position: fixed; bottom: 6px; left: 8px; z-index: 1100;
      background: rgba(13,17,23,0.75);
      border: 1px solid rgba(137,180,250,0.2);
      border-radius: 4px; padding: 4px 8px;
      font: 10px "Courier New", monospace;
      color: #a6e3a1; text-transform: uppercase;
      pointer-events: none;
    }
  </style>
</head>
<body>
  <div id="offlineBanner">OFFLINE - MAP TILES UNAVAILABLE</div>
  <div id="map"></div>
  <div id="radarHud"><canvas id="hudCanvas" width="180" height="220"></canvas></div>
  <div id="coordOverlay">LAT ---.------ LON ---.------</div>
  <script src="https://unpkg.com/maplibre-gl@4.7.1/dist/maplibre-gl.js"></script>
  <script>
    var status = {
      engine: "MapLibre GL",
      ready: false,
      interactive: false,
      online: navigator.onLine
    };

    function setOfflineBanner(online) {
      document.getElementById("offlineBanner").style.display = online ? "none" : "block";
      status.online = !!online;
    }
    window.addEventListener("online", function() { setOfflineBanner(true); });
    window.addEventListener("offline", function() { setOfflineBanner(false); });
    setOfflineBanner(navigator.onLine);

    var platformState = {
      lat: 32.0853, lon: 34.7818,
      heading: 0, hfov: 24,
      pitch: 0, vfov: 14,
      nearDist: 100, centerDist: 600, farDist: 1200
    };

    var styleSpec = {
      version: 8,
      glyphs: "https://demotiles.maplibre.org/font/{fontstack}/{range}.pbf",
      sources: {
        terrainSource: {
          type: "raster",
          tiles: ["https://tile.openstreetmap.org/{z}/{x}/{y}.png"],
          tileSize: 256
        },
        satelliteSource: {
          type: "raster",
          tiles: ["https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"],
          tileSize: 256
        },
        overlaySource: {
          type: "geojson",
          data: { type: "FeatureCollection", features: [] }
        },
        scanSource: {
          type: "geojson",
          data: { type: "FeatureCollection", features: [] }
        }
      },
      layers: [
        { id: "terrain-base", type: "raster", source: "terrainSource" },
        { id: "satellite-base", type: "raster", source: "satelliteSource", layout: { visibility: "none" } },
        {
          id: "fov-fill-far",
          type: "fill",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "fov"],
          paint: { "fill-color": "#f9e2af", "fill-opacity": 0.06 }
        },
        {
          id: "fov-fill-mid",
          type: "fill",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "fov-mid"],
          paint: { "fill-color": "#f9e2af", "fill-opacity": 0.18 }
        },
        {
          id: "fov-fill-near",
          type: "fill",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "fov-near"],
          paint: { "fill-color": "#f9e2af", "fill-opacity": 0.38 }
        },
        {
          id: "fov-fill-vnear",
          type: "fill",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "fov-vnear"],
          paint: { "fill-color": "#fab387", "fill-opacity": 0.58 }
        },
        {
          id: "fov-outline",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "fov"],
          paint: { "line-color": "#fab387", "line-width": 2.5, "line-opacity": 0.95 }
        },
        {
          id: "scan-preview-fill",
          type: "fill",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "scan-preview"],
          paint: { "fill-color": "#89b4fa", "fill-opacity": 0.22 }
        },
        {
          id: "scan-preview-line",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "scan-preview"],
          paint: { "line-color": "#89b4fa", "line-width": 3, "line-dasharray": [2, 2] }
        },
        {
          id: "fov-near-arc",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "near"],
          paint: { "line-color": "#fab387", "line-width": 2, "line-dasharray": [2, 2] }
        },
        {
          id: "fov-center-arc",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "center"],
          paint: { "line-color": "#f9e2af", "line-width": 1.5, "line-dasharray": [3, 3] }
        },
        {
          id: "fov-far-arc",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "far"],
          paint: { "line-color": "#89b4fa", "line-width": 2, "line-dasharray": [1, 2] }
        },
        {
          id: "scan-lines",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "scanline"],
          paint: { "line-color": "#f9e2af", "line-width": 1, "line-opacity": 0.10 }
        },
        {
          id: "heading-line",
          type: "line",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "heading"],
          paint: { "line-color": "#a6e3a1", "line-width": 3 }
        },
        {
          id: "platform-point",
          type: "circle",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "platform"],
          paint: {
            "circle-color": "#89b4fa", "circle-radius": 7,
            "circle-stroke-color": "#ffffff", "circle-stroke-width": 1.2
          }
        },
        {
          id: "target-point",
          type: "circle",
          source: "overlaySource",
          filter: ["==", ["get", "kind"], "target"],
          paint: {
            "circle-color": "#f38ba8", "circle-radius": 5,
            "circle-stroke-color": "#ffffff", "circle-stroke-width": 1.0
          }
        },
        {
          id: "scan-trail",
          type: "circle",
          source: "scanSource",
          filter: ["==", ["get", "kind"], "trail"],
          paint: {
            "circle-color": "#a6e3a1", "circle-radius": 2,
            "circle-opacity": 0.6
          }
        },
        {
          id: "scan-beam",
          type: "circle",
          source: "scanSource",
          filter: ["==", ["get", "kind"], "beam"],
          paint: {
            "circle-color": "#f9e2af", "circle-radius": 5,
            "circle-stroke-color": "#f9e2af", "circle-stroke-width": 2,
            "circle-opacity": 0.9
          }
        }
      ]
    };

    var map = new maplibregl.Map({
      container: "map",
      style: styleSpec,
      center: [34.7818, 32.0853],
      zoom: 12,
      pitch: 52,
      bearing: 0,
      antialias: true
    });
    map.addControl(new maplibregl.NavigationControl(), "top-right");
    var lastLon = 34.7818;
    var lastLat = 32.0853;

    function destinationPoint(lat, lon, bearingDeg, distanceMeters) {
      var R = 6378137.0;
      var br = bearingDeg * Math.PI / 180.0;
      var la = lat * Math.PI / 180.0;
      var lo = lon * Math.PI / 180.0;
      var d = distanceMeters / R;
      var sla = Math.sin(la), cla = Math.cos(la);
      var sd = Math.sin(d), cd = Math.cos(d);
      var la2 = Math.asin(sla * cd + cla * sd * Math.cos(br));
      var lo2 = lo + Math.atan2(Math.sin(br) * sd * cla, cd - sla * Math.sin(la2));
      return [lo2 * 180.0 / Math.PI, la2 * 180.0 / Math.PI];
    }

    function buildArc(lat, lon, headingDeg, hfovDeg, distanceMeters) {
      var half = Math.max(0.5, hfovDeg / 2.0);
      var start = headingDeg - half;
      var end = headingDeg + half;
      var step = Math.max(1.0, hfovDeg / 16.0);
      var points = [];
      for (var b = start; b <= end + 0.001; b += step) {
        points.push(destinationPoint(lat, lon, b, distanceMeters));
      }
      return points;
    }

    function buildFovPolygon(lat, lon, headingDeg, hfovDeg, nearDistance, farDistance) {
      var outer = buildArc(lat, lon, headingDeg, hfovDeg, farDistance);
      var inner = buildArc(lat, lon, headingDeg, hfovDeg, nearDistance).reverse();
      return outer.concat(inner).concat([[lon, lat]]);
    }

    function buildBandPolygon(lat, lon, headingDeg, hfovDeg, innerDist, outerDist) {
      var outer = buildArc(lat, lon, headingDeg, hfovDeg, outerDist);
      var inner = buildArc(lat, lon, headingDeg, hfovDeg, innerDist).reverse();
      return outer.concat(inner).concat([outer[0]]);
    }

    function makeLine(coords, kind) {
      return { type: "Feature", properties: { kind: kind }, geometry: { type: "LineString", coordinates: coords } };
    }
    function makePoint(coord, kind) {
      return { type: "Feature", properties: { kind: kind }, geometry: { type: "Point", coordinates: coord } };
    }
    function makePolygon(coords, kind) {
      return { type: "Feature", properties: { kind: kind }, geometry: { type: "Polygon", coordinates: [coords] } };
    }

    /* ---- Radar HUD overlay ---- */
    function drawRadarHud() {
      var cv = document.getElementById("hudCanvas");
      var ctx = cv.getContext("2d");
      var W = 180, H = 220;
      ctx.clearRect(0, 0, W, H);
      var s = platformState;
      var cx = 80, cy = 85, r = 68;

      ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI);
      ctx.strokeStyle = "rgba(137,180,250,0.35)"; ctx.lineWidth = 1; ctx.stroke();
      ctx.beginPath(); ctx.arc(cx, cy, r * 0.5, 0, 2 * Math.PI);
      ctx.strokeStyle = "rgba(137,180,250,0.15)"; ctx.stroke();

      for (var d = 0; d < 360; d += 30) {
        var rad = (d - 90) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(rad) * (r - 6), cy + Math.sin(rad) * (r - 6));
        ctx.lineTo(cx + Math.cos(rad) * r, cy + Math.sin(rad) * r);
        ctx.strokeStyle = d % 90 === 0 ? "rgba(205,214,244,0.7)" : "rgba(205,214,244,0.35)";
        ctx.lineWidth = d % 90 === 0 ? 1.5 : 1; ctx.stroke();
      }

      ctx.font = "bold 10px Courier New"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillStyle = "#cdd6f4";
      ctx.fillText("N", cx, cy - r - 8); ctx.fillText("S", cx, cy + r + 8);
      ctx.fillText("E", cx + r + 10, cy); ctx.fillText("W", cx - r - 10, cy);

      var hRad = (s.heading - 90) * Math.PI / 180;
      var halfF = s.hfov / 2 * Math.PI / 180;
      ctx.beginPath(); ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, r - 10, hRad - halfF, hRad + halfF); ctx.closePath();
      ctx.fillStyle = "rgba(249,226,175,0.25)"; ctx.fill();
      ctx.strokeStyle = "rgba(249,226,175,0.6)"; ctx.lineWidth = 1; ctx.stroke();

      var hx = cx + Math.cos(hRad) * (r + 2), hy = cy + Math.sin(hRad) * (r + 2);
      ctx.save(); ctx.shadowColor = "#a6e3a1"; ctx.shadowBlur = 6;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy);
      ctx.strokeStyle = "#a6e3a1"; ctx.lineWidth = 2; ctx.stroke(); ctx.restore();

      ctx.font = "10px Courier New"; ctx.fillStyle = "#a6e3a1"; ctx.textAlign = "center";
      ctx.fillText("HDG " + s.heading.toFixed(1) + "\u00b0", cx, cy + r + 20);

      var ex = 162, ey = 20, eH = 120, eW = 12;
      ctx.strokeStyle = "rgba(137,180,250,0.3)"; ctx.lineWidth = 1; ctx.strokeRect(ex, ey, eW, eH);
      var hzY = ey + eH / 2;
      ctx.beginPath(); ctx.moveTo(ex - 2, hzY); ctx.lineTo(ex + eW + 2, hzY);
      ctx.strokeStyle = "rgba(205,214,244,0.25)"; ctx.stroke();
      var pNorm = (-s.pitch + 90) / 180, pY = ey + pNorm * eH;
      var vN = s.vfov / 180 * eH;
      ctx.fillStyle = "rgba(249,226,175,0.2)"; ctx.fillRect(ex, pY - vN / 2, eW, vN);
      ctx.beginPath(); ctx.moveTo(ex - 2, pY); ctx.lineTo(ex + eW + 2, pY);
      ctx.strokeStyle = "#f9e2af"; ctx.lineWidth = 2; ctx.stroke();
      ctx.font = "9px Courier New"; ctx.fillStyle = "#f9e2af"; ctx.textAlign = "center";
      ctx.fillText("EL", ex + eW / 2, ey - 6);
      ctx.fillText(s.pitch.toFixed(1) + "\u00b0", ex + eW / 2, ey + eH + 12);
    }

    function updateCoordOverlay() {
      document.getElementById("coordOverlay").textContent =
        "LAT " + platformState.lat.toFixed(6) + " LON " + platformState.lon.toFixed(6);
    }

    /* ---- Map API (preserved call signatures) ---- */
    window.setMapLayer = function(mode) {
      var sat = (mode || "").toLowerCase().startsWith("sat");
      map.setLayoutProperty("terrain-base", "visibility", sat ? "none" : "visible");
      map.setLayoutProperty("satellite-base", "visibility", sat ? "visible" : "none");
    };

    window.setMapZoom = function(level) {
      if (Number.isFinite(level)) map.zoomTo(level, { duration: 250 });
    };

    window.centerPlatform = function() {
      map.easeTo({ center: [lastLon, lastLat], duration: 300 });
    };

    window.updatePlatform = function(payload) {
      if (!payload || !map.getSource("overlaySource")) return;
      var lat = Number(payload.lat);
      var lon = Number(payload.lon);
      var heading = Number(payload.heading);
      var hfov = Number(payload.hfov);
      var nearDistance = Math.max(10.0, Number(payload.nearDistance || 100.0));
      var centerDistance = Math.max(10.0, Number(payload.centerDistance || 120.0));
      var farDistance = Math.max(nearDistance + 5.0, Number(payload.farDistance || 200.0));
      lastLat = lat; lastLon = lon;
      platformState.lat = lat; platformState.lon = lon;
      platformState.heading = heading; platformState.hfov = hfov;
      platformState.pitch = Number(payload.pitch || 0);
      platformState.vfov = Number(payload.vfov || 14);
      platformState.nearDist = nearDistance;
      platformState.centerDist = centerDistance;
      platformState.farDist = farDistance;

      var target = destinationPoint(lat, lon, heading, centerDistance);
      var nearArc = buildArc(lat, lon, heading, hfov, nearDistance);
      var centerArcPts = buildArc(lat, lon, heading, hfov, centerDistance);
      var farArc = buildArc(lat, lon, heading, hfov, farDistance);
      var fovPolygon = buildFovPolygon(lat, lon, heading, hfov, nearDistance, farDistance);

      var span = farDistance - nearDistance;
      var fovMidBand = buildBandPolygon(lat, lon, heading, hfov, nearDistance, nearDistance + span * 0.45);
      var fovNearBand = buildBandPolygon(lat, lon, heading, hfov, nearDistance, nearDistance + span * 0.25);
      var fovVNearBand = buildBandPolygon(lat, lon, heading, hfov, nearDistance, nearDistance + span * 0.12);

      var features = [
        makePolygon(fovPolygon, "fov"),
        makePolygon(fovMidBand, "fov-mid"),
        makePolygon(fovNearBand, "fov-near"),
        makePolygon(fovVNearBand, "fov-vnear"),
        makeLine([[lon, lat], target], "heading"),
        makeLine(nearArc, "near"),
        makeLine(centerArcPts, "center"),
        makeLine(farArc, "far"),
        makePoint([lon, lat], "platform"),
        makePoint(target, "target")
      ];

      /* scan lines inside the FOV cone */
      for (var i = 0; i <= 4; i++) {
        var a = heading - hfov / 2 + hfov * i / 4;
        var fp = destinationPoint(lat, lon, a, farDistance);
        features.push(makeLine([[lon, lat], fp], "scanline"));
      }

      if (window._scanPreviewPoly) {
        features.push(makePolygon(window._scanPreviewPoly, "scan-preview"));
      }

      window._lastMapPayload = payload;
      map.getSource("overlaySource").setData({ type: "FeatureCollection", features: features });
      map.setBearing(heading);

      drawRadarHud();
      updateCoordOverlay();

      if (payload.follow) {
        map.easeTo({ center: [lon, lat], duration: 320 });
      }
    };

    /* ---- Scan progress overlay ---- */
    var scanTrailFeatures = [];

    window.updateScanProgress = function(payload) {
      if (!payload || !map.getSource("scanSource")) return;
      var el = Number(payload.el);
      var hdg = (payload.boresightHeading != null)
        ? Number(payload.boresightHeading)
        : (Number(payload.base_heading || 0) + Number(payload.az || 0));
      var dist = platformState.centerDist;
      var pt = destinationPoint(platformState.lat, platformState.lon, hdg, dist);
      scanTrailFeatures.push(makePoint(pt, "trail"));
      var features = scanTrailFeatures.slice();
      features.push(makePoint(pt, "beam"));
      map.getSource("scanSource").setData({ type: "FeatureCollection", features: features });
    };

    window.clearScanOverlay = function() {
      scanTrailFeatures = [];
      if (map.getSource("scanSource")) {
        map.getSource("scanSource").setData({ type: "FeatureCollection", features: [] });
      }
    };

    window._scanPreviewPoly = null;
    window.hideScanAreaPreview = function() {
      window._scanPreviewPoly = null;
      if (window._lastMapPayload) window.updatePlatform(window._lastMapPayload);
    };
    window.showScanAreaPreview = function(params) {
      if (!params) return;
      var lat = Number(params.lat), lon = Number(params.lon), alt = Number(params.alt || 120);
      var base = Number(params.base_heading || 0);
      var az0 = Number(params.az_start), az1 = Number(params.az_end);
      var el0 = Number(params.el_start), el1 = Number(params.el_end);
      var azMid = base + (az0 + az1) / 2, azSpan = Math.abs(az1 - az0);
      var distTop = alt / Math.tan(Math.max(0.8, Math.abs(el0)) * Math.PI / 180);
      var distBot = alt / Math.tan(Math.max(0.8, Math.abs(el1)) * Math.PI / 180);
      var arcTop = buildArc(lat, lon, azMid, Math.max(azSpan, 10), distTop);
      var arcBot = buildArc(lat, lon, azMid, Math.max(azSpan, 10), distBot).reverse();
      window._scanPreviewPoly = arcTop.concat(arcBot);
      if (window._lastMapPayload) window.updatePlatform(window._lastMapPayload);
    };
    window.waitForMapReady = function(timeoutMs) {
      return new Promise(function(resolve) {
        if (status.ready) { resolve(true); return; }
        map.once("load", function() { resolve(true); });
        setTimeout(function() { resolve(status.ready); }, timeoutMs || 8000);
      });
    };
    window.setCameraView = function() {};

    map.on("load", function() {
      status.ready = true;
      status.interactive = true;
      window.setMapLayer("terrain");
    });

    window.getMapStatus = function() {
      return {
        engine: status.engine,
        ready: status.ready,
        interactive: status.interactive,
        online: navigator.onLine
      };
    };
  </script>
</body>
</html>
"""

    @staticmethod
    def _cesium_html() -> str:
        return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1.0" />
  <script>
    window.CESIUM_BASE_URL = "https://unpkg.com/cesium@1.117/Build/Cesium/";
  </script>
  <link href="https://unpkg.com/cesium@1.117/Build/Cesium/Widgets/widgets.css" rel="stylesheet" />
  <style>
    html, body, #map {
      margin: 0; height: 100%; width: 100%;
      background: #0d1117; overflow: hidden;
      font-family: "Courier New", monospace;
    }
    #offlineBanner {
      position: fixed; top: 8px; left: 50%; transform: translateX(-50%);
      background: rgba(85,107,47,0.92); color: #f9e2af;
      padding: 5px 14px; border-radius: 3px; z-index: 2000;
      font: bold 11px "Courier New", monospace;
      text-transform: uppercase; letter-spacing: 1px;
      display: none; border: 1px solid rgba(249,226,175,0.4);
    }
    #radarHud {
      position: fixed; top: 10px; right: 10px; z-index: 2100;
      background: rgba(13,17,23,0.85);
      border: 1px solid rgba(137,180,250,0.3);
      border-radius: 6px; padding: 8px; pointer-events: none;
    }
    #coordOverlay {
      position: fixed; bottom: 6px; left: 8px; z-index: 2100;
      background: rgba(13,17,23,0.75);
      border: 1px solid rgba(137,180,250,0.2);
      border-radius: 4px; padding: 4px 8px;
      font: 10px "Courier New", monospace;
      color: #a6e3a1; text-transform: uppercase;
      pointer-events: none;
    }
    #mapLegend {
      position: fixed; bottom: 6px; right: 8px; z-index: 2100;
      background: rgba(13,17,23,0.92);
      border: 1px solid rgba(137,180,250,0.45);
      border-radius: 6px; padding: 8px 10px;
      font: 10px "Courier New", monospace;
      color: #cdd6f4; line-height: 1.55;
      max-width: 260px;
      pointer-events: none;
    }
    #mapLegend .title {
      font-weight: bold; color: #89b4fa;
      text-transform: uppercase; letter-spacing: 1px;
      margin-bottom: 6px; font-size: 11px;
    }
    #mapLegend .row { display: flex; align-items: center; margin: 3px 0; }
    #mapLegend .sw {
      width: 14px; height: 10px; margin-right: 8px;
      border: 1px solid rgba(255,255,255,0.35); flex-shrink: 0;
    }
  </style>
</head>
<body>
  <div id="offlineBanner">OFFLINE - IMAGERY/TERRAIN LIMITED</div>
  <div id="map"></div>
  <div id="radarHud"><canvas id="hudCanvas" width="180" height="220"></canvas></div>
  <div id="coordOverlay">LAT ---.------ LON ---.------</div>
  <div id="mapLegend">
    <div class="title">Geometry legend</div>
    <div class="row"><span class="sw" style="background:#89b4fa"></span>Platform / ENU axes</div>
    <div class="row"><span class="sw" style="background:#ff8c00"></span>Camera FOV volume</div>
    <div class="row"><span class="sw" style="background:#ffcc00"></span>FOV edges / boresight</div>
    <div class="row"><span class="sw" style="background:#4da6ff"></span>Scan preview (continuous)</div>
    <div class="row"><span class="sw" style="background:#00ff88"></span>Active scan beam</div>
    <div class="row"><span class="sw" style="background:#f38ba8"></span>Aim point</div>
  </div>
  <script src="https://unpkg.com/cesium@1.117/Build/Cesium/Cesium.js"></script>
  <script>
    var status = {
      engine: "CesiumJS 3D",
      ready: false,
      interactive: false,
      online: navigator.onLine
    };

    function setOfflineBanner(online) {
      document.getElementById("offlineBanner").style.display = online ? "none" : "block";
      status.online = !!online;
    }
    window.addEventListener("online", function() { setOfflineBanner(true); });
    window.addEventListener("offline", function() { setOfflineBanner(false); });
    setOfflineBanner(navigator.onLine);

    var platformState = {
      lat: 32.0853, lon: 34.7818,
      heading: 0, hfov: 24,
      pitch: 0, vfov: 14,
      nearDist: 100, centerDist: 600, farDist: 1200
    };

    var viewer = new Cesium.Viewer("map", {
      animation: false,
      timeline: false,
      baseLayerPicker: false,
      geocoder: false,
      sceneModePicker: true,
      navigationHelpButton: false,
      infoBox: false,
      fullscreenButton: false,
      homeButton: false,
      selectionIndicator: false,
      shouldAnimate: true
    });
    viewer.scene.globe.depthTestAgainstTerrain = false;
    viewer.scene.globe.show = true;
    viewer.scene.skyAtmosphere.show = true;
    viewer.scene.fog.enabled = false;

    var camCtrl = viewer.scene.screenSpaceCameraController;
    camCtrl.enableRotate = true;
    camCtrl.enableTranslate = true;
    camCtrl.enableZoom = true;
    camCtrl.enableTilt = true;
    camCtrl.enableLook = true;
    camCtrl.rotateEventTypes = [
      Cesium.CameraEventType.LEFT_DRAG,
      Cesium.CameraEventType.PINCH
    ];
    camCtrl.tiltEventTypes = [Cesium.CameraEventType.RIGHT_DRAG];
    camCtrl.zoomEventTypes = [
      Cesium.CameraEventType.WHEEL,
      Cesium.CameraEventType.PINCH
    ];

    var terrainImagery = new Cesium.UrlTemplateImageryProvider({
      url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
      maximumLevel: 19,
      credit: new Cesium.Credit("OpenStreetMap")
    });
    var satelliteImagery = new Cesium.UrlTemplateImageryProvider({
      url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      maximumLevel: 19,
      credit: new Cesium.Credit("Esri")
    });

    viewer.imageryLayers.removeAll();
    var terrainLayer = viewer.imageryLayers.addImageryProvider(terrainImagery);
    var satelliteLayer = viewer.imageryLayers.addImageryProvider(satelliteImagery);
    satelliteLayer.show = false;

    var platformEntity = viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(34.7818, 32.0853, 120.0),
      point: {
        pixelSize: 10,
        color: Cesium.Color.fromCssColorString("#89b4fa"),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 2
      },
      label: {
        text: "PLATFORM",
        font: "bold 12px Courier New",
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        fillColor: Cesium.Color.fromCssColorString("#d7e3fc"),
        outlineColor: Cesium.Color.BLACK,
        outlineWidth: 2,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -16)
      }
    });
    var targetEntity = viewer.entities.add({
      position: Cesium.Cartesian3.fromDegrees(34.79, 32.09, 0.0),
      point: {
        pixelSize: 8,
        color: Cesium.Color.fromCssColorString("#f38ba8"),
        outlineColor: Cesium.Color.WHITE,
        outlineWidth: 1.5
      }
    });
    var headingEntity = viewer.entities.add({
      polyline: {
        positions: [],
        width: 3,
        material: Cesium.Color.fromCssColorString("#a6e3a1")
      }
    });
    var beamEntity = viewer.entities.add({
      polyline: {
        positions: [],
        width: 2,
        material: Cesium.Color.fromCssColorString("#89b4fa").withAlpha(0.8)
      }
    });

    /* 3D sensor frustum in local ENU (gimbal = origin, yaw/pitch + camera HFoV/VFoV) */
    var fovSliceEntities = [];
    var fovEdgeEntities = [];
    var fovLabelEntities = [];
    var NUM_FOV_SLICES = 8;
    var SLICE_ALPHAS = [0.82, 0.68, 0.54, 0.40, 0.28, 0.18, 0.10, 0.05];
    var SLICE_COLOR = "#ff8c00";
    var EDGE_COLOR = "#ffcc00";
    var showFovFill = true;

    var axisEntities = [];
    function updateOriginAxes(lat, lon, alt) {
      axisEntities.forEach(function(e) { viewer.entities.remove(e); });
      axisEntities = [];
      var o = Cesium.Cartesian3.fromDegrees(lon, lat, alt);
      var len = Math.max(80, platformState.nearDist * 0.35);
      var dirs = [
        { enu: [len, 0, 0], color: "#f38ba8", label: "E" },
        { enu: [0, len, 0], color: "#a6e3a1", label: "N" },
        { enu: [0, 0, len], color: "#89b4fa", label: "U" }
      ];
      dirs.forEach(function(d) {
        var p2 = enuToWorld(lat, lon, alt, d.enu[0], d.enu[1], d.enu[2]);
        axisEntities.push(viewer.entities.add({
          polyline: {
            positions: [o, p2],
            width: 2,
            material: Cesium.Color.fromCssColorString(d.color).withAlpha(0.95)
          }
        }));
      });
    }

    var activeFrustumEdges = null;
    var scanPreviewEntities = [];
    var scanPathEntity = null;
    var cameraViewMode = "globe";
    var orbitBearing = 45;
    var orbitPitch = -35;
    var orbitRange = 2500;
    var cesiumEngineReady = false;
    platformState.alt = 120;

    for (var si = 0; si < NUM_FOV_SLICES; si++) {
      fovSliceEntities.push(viewer.entities.add({
        polygon: {
          hierarchy: new Cesium.PolygonHierarchy([]),
          perPositionHeight: true,
          material: Cesium.Color.fromCssColorString(SLICE_COLOR).withAlpha(SLICE_ALPHAS[si]),
          outline: false
        }
      }));
    }
    for (var ei = 0; ei < 8; ei++) {
      fovEdgeEntities.push(viewer.entities.add({
        polyline: {
          positions: [],
          width: 3.5,
          material: Cesium.Color.fromCssColorString(EDGE_COLOR).withAlpha(0.98)
        }
      }));
    }

    var boresightEntity = viewer.entities.add({
      polyline: {
        positions: [],
        width: 4,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.35,
          color: Cesium.Color.fromCssColorString("#00ff88")
        })
      }
    });

    var lastLat = 32.0853;
    var lastLon = 34.7818;
    var lastHeading = 0.0;
    var lastZoom = 12;

    function destinationPoint(lat, lon, bearingDeg, distanceMeters) {
      var R = 6378137.0;
      var br = bearingDeg * Math.PI / 180.0;
      var la = lat * Math.PI / 180.0;
      var lo = lon * Math.PI / 180.0;
      var d = distanceMeters / R;
      var sla = Math.sin(la), cla = Math.cos(la);
      var sd = Math.sin(d), cd = Math.cos(d);
      var la2 = Math.asin(sla * cd + cla * sd * Math.cos(br));
      var lo2 = lo + Math.atan2(Math.sin(br) * sd * cla, cd - sla * Math.sin(la2));
      return [la2 * 180.0 / Math.PI, lo2 * 180.0 / Math.PI];
    }

    function buildArc(lat, lon, headingDeg, hfovDeg, distanceMeters) {
      var half = Math.max(0.5, hfovDeg / 2.0);
      var start = headingDeg - half;
      var end = headingDeg + half;
      var step = Math.max(1.0, hfovDeg / 16.0);
      var points = [];
      for (var b = start; b <= end + 0.001; b += step) {
        points.push(destinationPoint(lat, lon, b, distanceMeters));
      }
      return points;
    }

    function buildFovPolygon(lat, lon, headingDeg, hfovDeg, nearDistance, farDistance) {
      var outer = buildArc(lat, lon, headingDeg, hfovDeg, farDistance);
      var inner = buildArc(lat, lon, headingDeg, hfovDeg, nearDistance).reverse();
      return outer.concat(inner);
    }

    function enuOffset(headingDeg, pitchDeg, rangeM, deltaYawDeg, deltaPitchDeg) {
      var br = Cesium.Math.toRadians(headingDeg + deltaYawDeg);
      var el = Cesium.Math.toRadians(pitchDeg + deltaPitchDeg);
      var cosEl = Math.cos(el);
      return {
        e: rangeM * cosEl * Math.sin(br),
        n: rangeM * cosEl * Math.cos(br),
        u: rangeM * Math.sin(el)
      };
    }

    function enuToWorld(lat, lon, alt, east, north, up) {
      var origin = Cesium.Cartesian3.fromDegrees(lon, lat, alt);
      var transform = Cesium.Transforms.eastNorthUpToFixedFrame(origin);
      var local = new Cesium.Cartesian3(east, north, up);
      return Cesium.Matrix4.multiplyByPoint(transform, local, new Cesium.Cartesian3());
    }

    function sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, rangeM) {
      var hf = hfov / 2.0;
      var vf = vfov / 2.0;
      var corners = [
        enuOffset(yaw, pitch, rangeM, -hf, -vf),
        enuOffset(yaw, pitch, rangeM, +hf, -vf),
        enuOffset(yaw, pitch, rangeM, +hf, +vf),
        enuOffset(yaw, pitch, rangeM, -hf, +vf)
      ];
      return corners.map(function(c) {
        return enuToWorld(lat, lon, alt, c.e, c.n, c.u);
      });
    }

    function clearFrustumLabels() {
      fovLabelEntities.forEach(function(e) { viewer.entities.remove(e); });
      fovLabelEntities = [];
    }

    /* Slant range along rays from altitude + elevation (pitch) in spherical model */
    function sphericalNearFar(alt, pitchDeg, vfovDeg) {
      var halfV = Math.max(0.5, vfovDeg / 2.0);
      var pitchDown = Math.max(0.8, Math.abs(pitchDeg));
      var depFar = Math.max(0.5, pitchDown - halfV) * Math.PI / 180.0;
      var depNear = Math.min(89.0, pitchDown + halfV) * Math.PI / 180.0;
      var farSlant = alt / Math.sin(depFar);
      var nearSlant = alt / Math.sin(depNear);
      if (nearSlant > farSlant) {
        var tmp = nearSlant;
        nearSlant = farSlant;
        farSlant = tmp;
      }
      nearSlant = Math.max(40.0, Math.min(60000.0, nearSlant));
      farSlant = Math.max(nearSlant + 20.0, Math.min(60000.0, farSlant));
      return { near: nearSlant, far: farSlant };
    }

    function updateFrustum3D(yaw, pitch, hfov, vfov, nearD, farD) {
      var lat = platformState.lat;
      var lon = platformState.lon;
      var alt = Math.max(10.0, platformState.alt || 120.0);
      var apex = Cesium.Cartesian3.fromDegrees(lon, lat, alt);
      var ranges = sphericalNearFar(alt, pitch, vfov);
      nearD = ranges.near;
      farD = ranges.far;
      platformState.nearDist = nearD;
      platformState.farDist = farD;

      for (var s = 0; s < NUM_FOV_SLICES; s++) {
        var t0 = s / NUM_FOV_SLICES;
        var t1 = (s + 1) / NUM_FOV_SLICES;
        var d0 = nearD + (farD - nearD) * t0;
        var d1 = nearD + (farD - nearD) * t1;
        var inner = sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, d0);
        var outer = sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, d1);
        fovSliceEntities[s].polygon.hierarchy = new Cesium.PolygonHierarchy(
          [inner[0], inner[1], inner[2], inner[3], outer[3], outer[2], outer[1], outer[0]]
        );
        var alpha = showFovFill ? SLICE_ALPHAS[s] : 0.0;
        fovSliceEntities[s].polygon.material = Cesium.Color.fromCssColorString(SLICE_COLOR)
          .withAlpha(alpha);
      }

      var nearC = sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, nearD);
      var farC = sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, farD);
      var edgeSets = [
        [apex, nearC[0]], [apex, nearC[1]], [apex, nearC[2]], [apex, nearC[3]],
        [nearC[0], farC[0]], [nearC[1], farC[1]], [nearC[2], farC[2]], [nearC[3], farC[3]]
      ];
      for (var e = 0; e < 8; e++) {
        fovEdgeEntities[e].polyline.positions = edgeSets[e];
      }

      var boreEnd = enuToWorld(lat, lon, alt,
        enuOffset(yaw, pitch, farD, 0, 0).e,
        enuOffset(yaw, pitch, farD, 0, 0).n,
        enuOffset(yaw, pitch, farD, 0, 0).u
      );
      boresightEntity.polyline.positions = [apex, boreEnd];

      clearFrustumLabels();
      var hf = hfov / 2.0;
      var vf = vfov / 2.0;
      var labelSpecs = [
        { dYaw: -hf, dPitch: 0, text: "YAW " + (yaw - hf).toFixed(1) + "\u00b0" },
        { dYaw: +hf, dPitch: 0, text: "YAW " + (yaw + hf).toFixed(1) + "\u00b0" },
        { dYaw: 0, dPitch: -vf, text: "PITCH " + (pitch - vf).toFixed(1) + "\u00b0" },
        { dYaw: 0, dPitch: +vf, text: "PITCH " + (pitch + vf).toFixed(1) + "\u00b0" },
        { dYaw: 0, dPitch: 0, text: "HFoV " + hfov.toFixed(1) + "\u00b0  VFoV " + vfov.toFixed(1) + "\u00b0" }
      ];
      labelSpecs.forEach(function(spec) {
        var c = enuOffset(yaw, pitch, farD * 0.55, spec.dYaw, spec.dPitch);
        fovLabelEntities.push(viewer.entities.add({
          position: enuToWorld(lat, lon, alt, c.e, c.n, c.u),
          label: {
            text: spec.text,
            font: "bold 13px Courier New",
            fillColor: Cesium.Color.fromCssColorString("#ffeeaa"),
            outlineColor: Cesium.Color.BLACK,
            outlineWidth: 3,
            style: Cesium.LabelStyle.FILL_AND_OUTLINE,
            verticalOrigin: Cesium.VerticalOrigin.CENTER
          }
        }));
      });

      updateOriginAxes(lat, lon, alt);
    }

    function zoomToHeight(zoomLevel) {
      return 28000000.0 / Math.pow(2, Math.max(1.0, zoomLevel - 1.0));
    }

    function platformCartesian() {
      return Cesium.Cartesian3.fromDegrees(
        lastLon, lastLat, Math.max(10.0, platformState.alt || 120.0)
      );
    }

    function applyOrbitCamera(durationSec) {
      var pos = platformCartesian();
      var range = Math.max(150.0, orbitRange);
      viewer.camera.flyToBoundingSphere(
        new Cesium.BoundingSphere(pos, 80.0),
        {
          duration: durationSec || 0.5,
          offset: new Cesium.HeadingPitchRange(
            Cesium.Math.toRadians(lastHeading + orbitBearing),
            Cesium.Math.toRadians(orbitPitch),
            range
          )
        }
      );
    }

    function applySensorPovCamera() {
      var pos = platformCartesian();
      var hdg = Cesium.Math.toRadians(platformState.heading);
      var pitchRad = Cesium.Math.toRadians(platformState.pitch);
      viewer.camera.setView({
        destination: pos,
        orientation: { heading: hdg, pitch: pitchRad, roll: 0.0 }
      });
      updateFrustum3D(
        platformState.heading, platformState.pitch,
        platformState.hfov, platformState.vfov,
        platformState.nearDist, platformState.farDist
      );
    }

    function centerCamera(durationSec) {
      if (cameraViewMode === "sensor") {
        applySensorPovCamera();
        return;
      }
      if (cameraViewMode === "orbit") {
        applyOrbitCamera(durationSec);
        return;
      }
      applyOrbitCamera(durationSec);
    }

    /* ---- Radar HUD overlay ---- */
    function drawRadarHud() {
      var cv = document.getElementById("hudCanvas");
      var ctx = cv.getContext("2d");
      var W = 180, H = 220;
      ctx.clearRect(0, 0, W, H);
      var s = platformState;
      var cx = 80, cy = 85, r = 68;

      ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2 * Math.PI);
      ctx.strokeStyle = "rgba(137,180,250,0.35)"; ctx.lineWidth = 1; ctx.stroke();
      ctx.beginPath(); ctx.arc(cx, cy, r * 0.5, 0, 2 * Math.PI);
      ctx.strokeStyle = "rgba(137,180,250,0.15)"; ctx.stroke();

      for (var d = 0; d < 360; d += 30) {
        var rad = (d - 90) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cx + Math.cos(rad) * (r - 6), cy + Math.sin(rad) * (r - 6));
        ctx.lineTo(cx + Math.cos(rad) * r, cy + Math.sin(rad) * r);
        ctx.strokeStyle = d % 90 === 0 ? "rgba(205,214,244,0.7)" : "rgba(205,214,244,0.35)";
        ctx.lineWidth = d % 90 === 0 ? 1.5 : 1; ctx.stroke();
      }

      ctx.font = "bold 10px Courier New"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
      ctx.fillStyle = "#cdd6f4";
      ctx.fillText("N", cx, cy - r - 8); ctx.fillText("S", cx, cy + r + 8);
      ctx.fillText("E", cx + r + 10, cy); ctx.fillText("W", cx - r - 10, cy);

      var hRad = (s.heading - 90) * Math.PI / 180;
      var halfF = s.hfov / 2 * Math.PI / 180;
      ctx.beginPath(); ctx.moveTo(cx, cy);
      ctx.arc(cx, cy, r - 10, hRad - halfF, hRad + halfF); ctx.closePath();
      ctx.fillStyle = "rgba(249,226,175,0.25)"; ctx.fill();
      ctx.strokeStyle = "rgba(249,226,175,0.6)"; ctx.lineWidth = 1; ctx.stroke();

      var hx = cx + Math.cos(hRad) * (r + 2), hy = cy + Math.sin(hRad) * (r + 2);
      ctx.save(); ctx.shadowColor = "#a6e3a1"; ctx.shadowBlur = 6;
      ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(hx, hy);
      ctx.strokeStyle = "#a6e3a1"; ctx.lineWidth = 2; ctx.stroke(); ctx.restore();

      ctx.font = "10px Courier New"; ctx.fillStyle = "#a6e3a1"; ctx.textAlign = "center";
      ctx.fillText("HDG " + s.heading.toFixed(1) + "\u00b0", cx, cy + r + 20);

      var ex = 162, ey = 20, eH = 120, eW = 12;
      ctx.strokeStyle = "rgba(137,180,250,0.3)"; ctx.lineWidth = 1; ctx.strokeRect(ex, ey, eW, eH);
      var hzY = ey + eH / 2;
      ctx.beginPath(); ctx.moveTo(ex - 2, hzY); ctx.lineTo(ex + eW + 2, hzY);
      ctx.strokeStyle = "rgba(205,214,244,0.25)"; ctx.stroke();
      var pNorm = (-s.pitch + 90) / 180, pY = ey + pNorm * eH;
      var vN = s.vfov / 180 * eH;
      ctx.fillStyle = "rgba(249,226,175,0.2)"; ctx.fillRect(ex, pY - vN / 2, eW, vN);
      ctx.beginPath(); ctx.moveTo(ex - 2, pY); ctx.lineTo(ex + eW + 2, pY);
      ctx.strokeStyle = "#f9e2af"; ctx.lineWidth = 2; ctx.stroke();
      ctx.font = "9px Courier New"; ctx.fillStyle = "#f9e2af"; ctx.textAlign = "center";
      ctx.fillText("EL", ex + eW / 2, ey - 6);
      ctx.fillText(s.pitch.toFixed(1) + "\u00b0", ex + eW / 2, ey + eH + 12);
    }

    function updateCoordOverlay() {
      document.getElementById("coordOverlay").textContent =
        "LAT " + platformState.lat.toFixed(6) + " LON " + platformState.lon.toFixed(6);
    }

    /* ---- Map API (preserved call signatures) ---- */
    window.setMapLayer = function(mode) {
      var sat = (mode || "").toLowerCase().startsWith("sat");
      terrainLayer.show = !sat;
      satelliteLayer.show = sat;
    };

    window.setMapZoom = function(level) {
      if (!Number.isFinite(level)) return;
      lastZoom = level;
      centerCamera(0.45);
    };

    window.centerPlatform = function() {
      centerCamera(0.55);
    };

    window.setCameraView = function(cfg) {
      if (!cfg) return;
      cameraViewMode = cfg.mode || "globe";
      if (cfg.bearing !== undefined) orbitBearing = Number(cfg.bearing);
      if (cfg.pitch !== undefined) orbitPitch = Number(cfg.pitch);
      if (cfg.range !== undefined) orbitRange = Number(cfg.range);
      if (cameraViewMode === "sensor") {
        applySensorPovCamera();
      } else {
        applyOrbitCamera(0.45);
      }
    };

    window.updatePlatform = function(payload) {
      if (!payload) return;

      var lat = Number(payload.lat);
      var lon = Number(payload.lon);
      var heading = Number(payload.heading);
      var altitude = Math.max(10.0, Number(payload.alt || 50.0));
      var hfov = Number(payload.hfov);
      var nearDistance = Math.max(10.0, Number(payload.nearDistance || 100.0));
      var centerDistance = Math.max(10.0, Number(payload.centerDistance || 120.0));
      var farDistance = Math.max(nearDistance + 5.0, Number(payload.farDistance || 200.0));

      lastLat = lat; lastLon = lon; lastHeading = heading;
      platformState.lat = lat; platformState.lon = lon;
      platformState.heading = heading; platformState.hfov = hfov;
      platformState.pitch = Number(payload.pitch || 0);
      platformState.vfov = Number(payload.vfov || 14);
      platformState.alt = altitude;
      platformState.nearDist = nearDistance;
      platformState.centerDist = centerDistance;
      platformState.farDist = farDistance;

      var target = destinationPoint(lat, lon, heading, centerDistance);

      updateFrustum3D(heading, platformState.pitch, hfov, platformState.vfov, nearDistance, farDistance);

      var platformPos = Cesium.Cartesian3.fromDegrees(lon, lat, altitude);
      var boreTip = enuToWorld(lat, lon, altitude,
        enuOffset(heading, platformState.pitch, centerDistance, 0, 0).e,
        enuOffset(heading, platformState.pitch, centerDistance, 0, 0).n,
        enuOffset(heading, platformState.pitch, centerDistance, 0, 0).u
      );
      var targetPos = boreTip;
      platformEntity.position = platformPos;
      targetEntity.position = targetPos;

      headingEntity.polyline.positions = [platformPos, targetPos];
      beamEntity.polyline.positions = [platformPos, targetPos];

      if (payload.label) {
        platformEntity.label.text = payload.label;
      }

      drawRadarHud();
      updateCoordOverlay();

      if (payload.follow) {
        centerCamera(0.4);
      }
    };

    /* ---- 3D scan: continuous swept volume (one wedge per elevation row) ---- */
    var scanTrailEntities = [];
    var activeScanFrustumEdges = [];
    var activeScanFrustumFill = [];

    function clearActiveScanFrustum() {
      activeScanFrustumEdges.forEach(function(e) { viewer.entities.remove(e); });
      activeScanFrustumFill.forEach(function(e) { viewer.entities.remove(e); });
      activeScanFrustumEdges = [];
      activeScanFrustumFill = [];
    }

    function quadAt(lat, lon, alt, yawL, yawR, pitch, hfov, vfov, rangeM) {
      var hf = hfov / 2.0;
      var vf = vfov / 2.0;
      return [
        enuToWorld(lat, lon, alt, enuOffset(yawL, pitch - vf, rangeM, 0, 0).e,
          enuOffset(yawL, pitch - vf, rangeM, 0, 0).n, enuOffset(yawL, pitch - vf, rangeM, 0, 0).u),
        enuToWorld(lat, lon, alt, enuOffset(yawR, pitch - vf, rangeM, 0, 0).e,
          enuOffset(yawR, pitch - vf, rangeM, 0, 0).n, enuOffset(yawR, pitch - vf, rangeM, 0, 0).u),
        enuToWorld(lat, lon, alt, enuOffset(yawR, pitch + vf, rangeM, 0, 0).e,
          enuOffset(yawR, pitch + vf, rangeM, 0, 0).n, enuOffset(yawR, pitch + vf, rangeM, 0, 0).u),
        enuToWorld(lat, lon, alt, enuOffset(yawL, pitch + vf, rangeM, 0, 0).e,
          enuOffset(yawL, pitch + vf, rangeM, 0, 0).n, enuOffset(yawL, pitch + vf, rangeM, 0, 0).u)
      ];
    }

    function addPolylineEdges(pts, colorHex, width, alpha, store) {
      var col = Cesium.Color.fromCssColorString(colorHex).withAlpha(alpha);
      for (var i = 0; i < pts.length; i++) {
        var j = (i + 1) % pts.length;
        store.push(viewer.entities.add({
          polyline: { positions: [pts[i], pts[j]], width: width, material: col }
        }));
      }
    }

    function loftRowToNext(nearA, farA, nearB, farB, colorHex, store) {
      var col = Cesium.Color.fromCssColorString(colorHex).withAlpha(0.85);
      for (var i = 0; i < 4; i++) {
        store.push(viewer.entities.add({
          polyline: { positions: [nearA[i], nearB[i]], width: 2.2, material: col }
        }));
        store.push(viewer.entities.add({
          polyline: { positions: [farA[i], farB[i]], width: 2.2, material: col }
        }));
      }
    }

    function buildScanElevationRows(params) {
      var az0 = Number(params.az_start);
      var az1 = Number(params.az_end);
      var el0 = Number(params.el_start);
      var el1 = Number(params.el_end);
      var vfov = Number(params.vfov || 14);
      var base = Number(params.base_heading || 0);
      var yawL = base + Math.min(az0, az1) - Number(params.hfov || 24) / 2.0;
      var yawR = base + Math.max(az0, az1) + Number(params.hfov || 24) / 2.0;
      var nPasses = Math.max(1, Math.ceil(Math.abs(el0 - el1) / Math.max(0.5, vfov)));
      var rows = [];
      for (var pass = 0; pass < nPasses; pass++) {
        var el = el0 - pass * vfov;
        if (el < el1) el = el1;
        rows.push({ yawL: yawL, yawR: yawR, pitch: el, pass: pass });
      }
      return rows;
    }

    function drawContinuousScanRow(lat, lon, alt, row, hfov, vfov, fillStore, edgeStore) {
      var ranges = sphericalNearFar(alt, row.pitch, vfov);
      var nearQ = quadAt(lat, lon, alt, row.yawL, row.yawR, row.pitch, hfov, vfov, ranges.near);
      var farQ = quadAt(lat, lon, alt, row.yawL, row.yawR, row.pitch, hfov, vfov, ranges.far);
      var apex = Cesium.Cartesian3.fromDegrees(lon, lat, alt);

      if (showFovFill) {
        fillStore.push(viewer.entities.add({
          polygon: {
            hierarchy: new Cesium.PolygonHierarchy(
              [nearQ[0], nearQ[1], nearQ[2], nearQ[3], farQ[3], farQ[2], farQ[1], farQ[0]]
            ),
            perPositionHeight: true,
            material: Cesium.Color.fromCssColorString("#4da6ff").withAlpha(0.14),
            outline: false
          }
        }));
      }

      addPolylineEdges(nearQ, "#6eb5ff", 2.8, 0.95, edgeStore);
      addPolylineEdges(farQ, "#4da6ff", 2.8, 0.95, edgeStore);
      for (var i = 0; i < 4; i++) {
        edgeStore.push(viewer.entities.add({
          polyline: {
            positions: [apex, farQ[i]],
            width: 1.6,
            material: Cesium.Color.fromCssColorString("#4da6ff").withAlpha(0.45)
          }
        }));
        edgeStore.push(viewer.entities.add({
          polyline: {
            positions: [nearQ[i], farQ[i]],
            width: 2.4,
            material: Cesium.Color.fromCssColorString("#89d4ff").withAlpha(0.9)
          }
        }));
      }
      return { nearQ: nearQ, farQ: farQ };
    }

    function drawActiveScanBeam(lat, lon, alt, yaw, pitch, hfov, vfov) {
      var ranges = sphericalNearFar(alt, pitch, vfov);
      var nearD = ranges.near;
      var farD = ranges.far;
      var apex = Cesium.Cartesian3.fromDegrees(lon, lat, alt);
      var nearC = sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, nearD);
      var farC = sliceCornersWorld(lat, lon, alt, yaw, pitch, hfov, vfov, farD);

      if (showFovFill) {
        activeScanFrustumFill.push(viewer.entities.add({
          polygon: {
            hierarchy: new Cesium.PolygonHierarchy(
              [nearC[0], nearC[1], nearC[2], nearC[3], farC[3], farC[2], farC[1], farC[0]]
            ),
            perPositionHeight: true,
            material: Cesium.Color.fromCssColorString("#00ff88").withAlpha(0.12),
            outline: false
          }
        }));
      }

      addPolylineEdges(nearC, "#00ff88", 3.2, 0.98, activeScanFrustumEdges);
      addPolylineEdges(farC, "#00ffaa", 3.2, 0.98, activeScanFrustumEdges);
      var edgePairs = [
        [apex, nearC[0]], [apex, nearC[1]], [apex, nearC[2]], [apex, nearC[3]],
        [nearC[0], farC[0]], [nearC[1], farC[1]], [nearC[2], farC[2]], [nearC[3], farC[3]]
      ];
      var edgeCol = Cesium.Color.fromCssColorString("#00ff88").withAlpha(0.95);
      edgePairs.forEach(function(pair) {
        activeScanFrustumEdges.push(viewer.entities.add({
          polyline: { positions: pair, width: 3.5, material: edgeCol }
        }));
      });
      return farD;
    }

    window.updateScanProgress = function(payload) {
      if (!payload) return;
      var lat = Number(payload.lat || platformState.lat);
      var lon = Number(payload.lon || platformState.lon);
      var alt = Math.max(10, Number(payload.alt || platformState.alt || 120));
      var el = Number(payload.el);
      var yaw = (payload.boresightHeading != null)
        ? Number(payload.boresightHeading)
        : (Number(payload.base_heading || 0) + Number(payload.az || 0));
      var hfov = Number(payload.hfov || platformState.hfov);
      var vfov = Number(payload.vfov || platformState.vfov);

      clearActiveScanFrustum();
      var farD = drawActiveScanBeam(lat, lon, alt, yaw, el, hfov, vfov);

      var tip = enuToWorld(lat, lon, alt,
        enuOffset(yaw, el, farD, 0, 0).e,
        enuOffset(yaw, el, farD, 0, 0).n,
        enuOffset(yaw, el, farD, 0, 0).u
      );
      scanTrailEntities.push(viewer.entities.add({
        position: tip,
        point: {
          pixelSize: 6,
          color: Cesium.Color.fromCssColorString("#00ff88"),
          outlineColor: Cesium.Color.BLACK,
          outlineWidth: 1
        }
      }));
      if (scanTrailEntities.length > 400) {
        viewer.entities.remove(scanTrailEntities.shift());
      }
    };

    window.clearScanOverlay = function() {
      scanTrailEntities.forEach(function(e) { viewer.entities.remove(e); });
      scanTrailEntities = [];
      clearActiveScanFrustum();
      window.hideScanAreaPreview();
    };

    window.hideScanAreaPreview = function() {
      scanPreviewEntities.forEach(function(e) { viewer.entities.remove(e); });
      scanPreviewEntities = [];
      if (scanPathEntity) {
        viewer.entities.remove(scanPathEntity);
        scanPathEntity = null;
      }
    };

    window.showScanAreaPreview = function(params) {
      if (!params) return;
      window.hideScanAreaPreview();
      var lat = Number(params.lat);
      var lon = Number(params.lon);
      var alt = Math.max(10, Number(params.alt || 120));
      var hfov = Number(params.hfov || 24);
      var vfov = Number(params.vfov || 14);

      var rows = buildScanElevationRows(params);
      var prev = null;
      var pathPts = [];
      rows.forEach(function(row) {
        var q = drawContinuousScanRow(lat, lon, alt, row, hfov, vfov, scanPreviewEntities, scanPreviewEntities);
        if (prev) {
          loftRowToNext(prev.nearQ, prev.farQ, q.nearQ, q.farQ, "#5a9fd4", scanPreviewEntities);
        }
        var midYaw = (row.yawL + row.yawR) / 2.0;
        var rowRanges = sphericalNearFar(alt, row.pitch, vfov);
        var c = enuOffset(midYaw, row.pitch, rowRanges.far * 0.92, 0, 0);
        pathPts.push(enuToWorld(lat, lon, alt, c.e, c.n, c.u));
        prev = q;
      });

      if (pathPts.length >= 2) {
        scanPathEntity = viewer.entities.add({
          polyline: {
            positions: pathPts,
            width: 3,
            material: new Cesium.PolylineGlowMaterialProperty({
              glowPower: 0.25,
              color: Cesium.Color.fromCssColorString("#4da6ff")
            })
          }
        });
      }
    };

    window.setLegendVisible = function(visible) {
      var el = document.getElementById("mapLegend");
      if (el) el.style.display = visible ? "block" : "none";
    };

    window.setFovFillVisible = function(visible) {
      showFovFill = !!visible;
      if (platformState.lat) {
        updateFrustum3D(
          platformState.heading, platformState.pitch,
          platformState.hfov, platformState.vfov,
          platformState.nearDist, platformState.farDist
        );
      }
    };

    window.waitForMapReady = function(timeoutMs) {
      timeoutMs = timeoutMs || 8000;
      return new Promise(function(resolve) {
        if (cesiumEngineReady) { resolve(true); return; }
        var done = false;
        var timer = setTimeout(function() {
          if (!done) { done = true; resolve(cesiumEngineReady); }
        }, timeoutMs);
        var remove = viewer.scene.globe.tileLoadProgressEvent.addEventListener(function(q) {
          if (q === 0 && !done) {
            done = true;
            clearTimeout(timer);
            cesiumEngineReady = true;
            status.ready = true;
            status.interactive = true;
            resolve(true);
          }
        });
        setTimeout(function() {
          if (!cesiumEngineReady) {
            cesiumEngineReady = true;
            status.ready = true;
            status.interactive = true;
          }
        }, 2500);
      });
    };

    cesiumEngineReady = true;
    status.ready = true;
    status.interactive = true;
    centerCamera(0.15);

    window.getMapStatus = function() {
      return {
        engine: status.engine,
        ready: status.ready && cesiumEngineReady,
        interactive: status.interactive && cesiumEngineReady,
        online: navigator.onLine
      };
    };
  </script>
</body>
</html>
"""

    def _reload_map_engine(self):
        if self.map_view is None:
            return
        self._map_load_id += 1
        self._map_loaded = False
        self._map_online = False
        self._map_interactive = False
        self._map_engine_name = self.map_engine.currentText()
        self._update_map_connectivity_status(detail="loading...")
        self.btn_reload_map.setEnabled(False)
        self._map_reload_timer.start()

        if QWebEnginePage is not None:
            self.map_view.page().triggerAction(QWebEnginePage.Stop)

        engine_key = self.map_engine.currentData()
        html = self._build_map_html(engine_key)
        if self._map_tmp_file:
            try:
                os.unlink(self._map_tmp_file)
            except OSError:
                pass
            self._map_tmp_file = None
        tmp = tempfile.NamedTemporaryFile(
            suffix=".html", delete=False, mode="w",
            encoding="utf-8",
            dir=os.path.dirname(os.path.abspath(__file__)),
        )
        tmp.write(html)
        tmp.close()
        self._map_tmp_file = tmp.name
        self.map_view.setUrl(QUrl.fromLocalFile(tmp.name))

    def _on_map_reload_timeout(self):
        if self._map_loaded:
            return
        self.btn_reload_map.setEnabled(True)
        self._map_interactive = False
        self._update_map_connectivity_status(detail="load timeout - click Reload")
        self._set_status("Map load timed out. Try Reload Engine again.")

    def _on_map_engine_changed(self, _index: int):
        self._reload_map_engine()

    def _on_map_loaded(self, ok: bool):
        self._map_reload_timer.stop()
        self.btn_reload_map.setEnabled(True)
        self._map_loaded = bool(ok)
        if not ok:
            self._map_online = False
            self._map_interactive = False
            self._update_map_connectivity_status(detail="failed to load")
            self._set_status("Map failed to load.")
            return

        def _after_engine_ready(ready: bool):
            self._map_interactive = bool(ready)
            self._map_online = bool(ready)
            self._on_map_layer_changed()
            self._on_map_zoom_changed(self.map_zoom.value())
            self._refresh_map_overlay(pan_map=False)
            self._poll_map_status()
            if self.map_engine.currentData() == "cesium":
                self._apply_cesium_camera()
            if hasattr(self, "map_show_legend"):
                self._on_map_legend_toggled(self.map_show_legend.isChecked())
                self._on_map_fill_toggled(self.map_show_fill.isChecked())

        self._run_map_js(
            "(window.waitForMapReady && window.waitForMapReady(8000)) || Promise.resolve(true);",
            lambda ready: _after_engine_ready(bool(ready)),
        )

    def _run_map_js(self, code: str, callback=None):
        if not self._map_loaded or self.map_view is None:
            return
        if callback is None:
            self.map_view.page().runJavaScript(code)
        else:
            self.map_view.page().runJavaScript(code, callback)

    def _poll_map_status(self):
        if not self._map_loaded:
            self._update_map_connectivity_status()
            return
        self._run_map_js(
            "(window.getMapStatus && window.getMapStatus()) || "
            "{engine: 'unknown', ready: false, interactive: false, online: false};",
            self._on_map_status_result,
        )

    def _on_map_status_result(self, result):
        if isinstance(result, dict):
            self._map_engine_name = str(result.get("engine", self.map_engine.currentText()))
            ready = bool(result.get("ready", False))
            self._map_interactive = ready and bool(result.get("interactive", False))
            self._map_online = ready and bool(result.get("online", False))
        self._update_map_connectivity_status()

    def _update_map_connectivity_status(self, detail: str = ""):
        if not hasattr(self, "lbl_map_online"):
            return

        if not WEBENGINE_AVAILABLE:
            self.map_online_indicator.set_color("#f38ba8")
            self.lbl_map_online.setProperty("class", "status-bad")
            self.lbl_map_online.setText("QWebEngine unavailable")
            self.lbl_map_online.style().unpolish(self.lbl_map_online)
            self.lbl_map_online.style().polish(self.lbl_map_online)
            return

        if not self._map_loaded:
            color = "#fab387"
            cls = "status-warn"
            text = f"{self._map_engine_name}: loading"
        elif self._map_online and self._map_interactive:
            color = "#a6e3a1"
            cls = "status-good"
            text = f"{self._map_engine_name}: online / interactive"
        elif self._map_interactive:
            color = "#fab387"
            cls = "status-warn"
            text = f"{self._map_engine_name}: loaded (offline)"
        else:
            color = "#f38ba8"
            cls = "status-bad"
            text = f"{self._map_engine_name}: not interactive"

        if detail:
            text = f"{text} | {detail}"

        self.map_online_indicator.set_color(color)
        self.lbl_map_online.setProperty("class", cls)
        self.lbl_map_online.setText(text)
        self.lbl_map_online.style().unpolish(self.lbl_map_online)
        self.lbl_map_online.style().polish(self.lbl_map_online)

    def _update_map_live_status(self):
        if not hasattr(self, "lbl_map_live"):
            return

        connected = self.controller is not None and self.controller.is_connected()
        polling = self._polling_timer.isActive()
        have_positions = (
            self._axis_positions_last_good[0] is not None
            and self._axis_positions_last_good[1] is not None
        )

        if connected and polling and have_positions:
            color = "#a6e3a1"
            cls = "status-good"
            text = "Live telemetry ON (yaw + pitch)"
        elif connected and polling:
            color = "#fab387"
            cls = "status-warn"
            text = "Connected, waiting for axis telemetry"
        elif connected:
            color = "#fab387"
            cls = "status-warn"
            text = "Controller connected (auto-read is OFF)"
        else:
            color = "#585b70"
            cls = ""
            text = "Controller disconnected"

        self.map_live_indicator.set_color(color)
        self.lbl_map_live.setProperty("class", cls)
        self.lbl_map_live.setText(text)
        self.lbl_map_live.style().unpolish(self.lbl_map_live)
        self.lbl_map_live.style().polish(self.lbl_map_live)

    def _on_map_layer_changed(self):
        mode = "satellite" if self.map_layer.currentText().lower().startswith("sat") else "terrain"
        self._run_map_js(f"window.setMapLayer({json.dumps(mode)});")

    def _on_map_zoom_changed(self, value: int):
        self._run_map_js(f"window.setMapZoom({int(value)});")

    def _computed_map_heading(self) -> float:
        heading = self.map_heading.value()
        yaw = self._axis_positions_last_good[0]
        if self.map_use_gimbal_heading.isChecked() and yaw is not None:
            heading += yaw
        return heading % 360.0

    def _schedule_map_overlay_refresh(self) -> None:
        self._map_overlay_timer.start()

    def _update_axis_display_labels(self) -> None:
        """Fast UI refresh for motor readouts (no map JS)."""
        yaw = self._axis_positions_last_good[0]
        pitch = self._axis_positions_last_good[1]
        if hasattr(self, "lbl_map_yaw"):
            self.lbl_map_yaw.setText("---" if yaw is None else f"{yaw:.2f}")
        if hasattr(self, "lbl_map_pitch"):
            self.lbl_map_pitch.setText("---" if pitch is None else f"{pitch:.2f}")
        if hasattr(self, "lbl_map_heading"):
            self.lbl_map_heading.setText(f"{self._computed_map_heading():.2f}")
        for idx in range(min(2, len(self.axis_panels))):
            val = self._axis_positions_last_good[idx]
            lbl = self.axis_panels[idx]["lbl_pos"]
            if val is not None:
                lbl.setText(f"{val:.2f}")

    def _io_busy(self) -> bool:
        self._prune_finished_workers()
        return any(w.isRunning() for w in self._workers)

    def _register_worker(self, worker: QThread) -> bool:
        self._prune_finished_workers()
        if len(self._workers) >= 4:
            return False
        worker.setParent(self)
        worker.finished.connect(self._on_io_thread_finished)
        self._workers.append(worker)
        return True

    def _prune_finished_workers(self) -> None:
        alive: list[QThread] = []
        for worker in self._workers:
            if worker.isRunning():
                alive.append(worker)
            else:
                worker.wait(200)
                worker.deleteLater()
        self._workers = alive

    def _on_io_thread_finished(self) -> None:
        worker = self.sender()
        if worker is None:
            return
        if worker in self._workers:
            self._workers.remove(worker)
        if worker is self._poll_worker:
            self._poll_worker = None
        QTimer.singleShot(0, worker.deleteLater)

    def _wait_for_io_threads(self, timeout_ms: int = 2500) -> None:
        self._polling_timer.stop()
        if self._poll_worker is not None:
            poll = self._poll_worker
            if poll.isRunning():
                poll.wait(min(timeout_ms, 1500))
            if poll is self._poll_worker:
                self._poll_worker = None
        for worker in list(self._workers):
            if worker.isRunning():
                worker.wait(max(200, timeout_ms // max(1, len(self._workers))))
            if worker in self._workers:
                self._workers.remove(worker)
            worker.deleteLater()
        self._workers.clear()

    def _refresh_map_overlay(self, pan_map: bool = False) -> None:
        self._update_axis_display_labels()
        yaw = self._axis_positions_last_good[0]
        pitch = self._axis_positions_last_good[1]
        heading = self._computed_map_heading()

        near_distance, center_distance, far_distance = self._compute_fov_distances(pitch)
        boresight_distance = center_distance
        self.lbl_map_footprint.setText(
            f"Footprint: near {near_distance:.0f} m | center {center_distance:.0f} m | far {far_distance:.0f} m"
        )

        payload = {
            "lat": self.map_lat.value(),
            "lon": self.map_lon.value(),
            "alt": self.map_alt.value(),
            "pitch": 0.0 if pitch is None else pitch,
            "heading": heading,
            "boresightHeading": heading,
            "baseHeading": self.map_heading.value(),
            "gimbalYaw": 0.0 if yaw is None else yaw,
            "boresightDistance": boresight_distance,
            "usePitchForRange": self.map_use_pitch_for_range.isChecked(),
            "hfov": self.map_hfov.value(),
            "vfov": self.map_vfov.value(),
            "nearDistance": near_distance,
            "centerDistance": center_distance,
            "farDistance": far_distance,
            "follow": bool(pan_map and self.map_follow.isChecked()),
            "label": (
                f"ALT {self.map_alt.value():.0f}m | HDG {heading:.1f}deg | "
                f"PITCH {(0.0 if pitch is None else pitch):.1f}deg"
            ),
            "targetLabel": f"Sight point {center_distance:.0f}m",
        }
        self._run_map_js(f"window.updatePlatform({json.dumps(payload)});")
        self._update_map_live_status()

    @staticmethod
    def _is_suspicious_position_jump(previous: float | None, new_value: float) -> bool:
        """Reject spurious zero reads when the axis was clearly elsewhere."""
        if previous is None:
            return False
        if abs(new_value) > 0.05:
            return False
        return abs(previous) > 2.0

    def _accept_axis_position(self, axis_idx: int, value: float | None) -> float | None:
        if value is None or not math.isfinite(value):
            return self._axis_positions_last_good[axis_idx]
        if self._is_suspicious_position_jump(self._axis_positions_last_good[axis_idx], value):
            return self._axis_positions_last_good[axis_idx]
        self._axis_positions[axis_idx] = value
        self._axis_positions_last_good[axis_idx] = value
        return value

    def _compute_fov_distances(self, pitch_value: float | None) -> tuple[float, float, float]:
        altitude = max(1.0, self.map_alt.value())
        center_distance = max(25.0, self.map_range.value())
        half_vertical = max(0.5, self.map_vfov.value() / 2.0)

        if self.map_use_pitch_for_range.isChecked() and pitch_value is not None:
            pitch_down = max(0.8, abs(pitch_value))
            top_angle = max(0.5, pitch_down - half_vertical)
            bottom_angle = min(89.0, pitch_down + half_vertical)

            far_distance = altitude / max(1e-6, math.sin(math.radians(top_angle)))
            near_distance = altitude / max(1e-6, math.sin(math.radians(bottom_angle)))
            if near_distance > far_distance:
                near_distance, far_distance = far_distance, near_distance

            near_distance = max(25.0, min(60000.0, near_distance))
            far_distance = max(near_distance + 10.0, min(60000.0, far_distance))
            center_distance = max(near_distance + 2.0, min(far_distance - 2.0, (near_distance + far_distance) / 2.0))
            return near_distance, center_distance, far_distance

        span = max(10.0, center_distance * math.tan(math.radians(half_vertical)))
        near_distance = max(25.0, center_distance - span)
        far_distance = max(near_distance + 10.0, min(60000.0, center_distance + span))
        return near_distance, center_distance, far_distance

    def _save_preset(self, preset_key: str):
        self._map_presets[preset_key] = {
            "lat": self.map_lat.value(),
            "lon": self.map_lon.value(),
            "alt": self.map_alt.value(),
            "heading": self.map_heading.value(),
        }
        self._persist_map_presets()
        self._refresh_preset_labels()
        self._set_status(f"{preset_key.replace('_', ' ').title()} saved.")

    def _load_preset(self, preset_key: str):
        preset = self._map_presets.get(preset_key)
        if not preset:
            self._set_status(f"{preset_key.replace('_', ' ').title()} is empty.")
            return

        self.map_lat.setValue(float(preset.get("lat", self.map_lat.value())))
        self.map_lon.setValue(float(preset.get("lon", self.map_lon.value())))
        self.map_alt.setValue(float(preset.get("alt", self.map_alt.value())))
        self.map_heading.setValue(float(preset.get("heading", self.map_heading.value())))
        self._schedule_map_overlay_refresh()
        self._set_status(f"{preset_key.replace('_', ' ').title()} loaded.")

    def _refresh_preset_labels(self):
        if not hasattr(self, "lbl_preset_1"):
            return

        def _preset_text(key: str, idx: int) -> str:
            item = self._map_presets.get(key)
            if not item:
                return f"Preset {idx}: not saved"
            return (
                f"Preset {idx}: "
                f"{item.get('lat', 0.0):.6f}, {item.get('lon', 0.0):.6f}, "
                f"ALT {item.get('alt', 0.0):.1f} m, HDG {item.get('heading', 0.0):.1f}deg"
            )

        self.lbl_preset_1.setText(_preset_text("preset_1", 1))
        self.lbl_preset_2.setText(_preset_text("preset_2", 2))
        self.btn_load_preset_1.setEnabled("preset_1" in self._map_presets)
        self.btn_load_preset_2.setEnabled("preset_2" in self._map_presets)

    def _load_map_presets(self):
        self._map_presets = {}
        if not os.path.exists(self._preset_file):
            return
        try:
            with open(self._preset_file, "r", encoding="utf-8") as f:
                loaded = json.load(f)
            if isinstance(loaded, dict):
                for key in ("preset_1", "preset_2"):
                    if key in loaded and isinstance(loaded[key], dict):
                        self._map_presets[key] = loaded[key]
        except Exception:
            self._map_presets = {}

    def _persist_map_presets(self):
        try:
            with open(self._preset_file, "w", encoding="utf-8") as f:
                json.dump(self._map_presets, f, indent=2)
        except Exception as e:
            self._set_status(f"Could not save presets: {e}")

    # -----------------------------------------------------------------
    #  Map sync + scan control
    # -----------------------------------------------------------------
    def _update_map_sync_label(self) -> None:
        if not hasattr(self, "lbl_map_sync_state"):
            return
        if self._map_sync_active:
            self.lbl_map_sync_state.setText("Live sync ON")
            self.lbl_map_sync_state.setProperty("class", "status-good")
        else:
            self.lbl_map_sync_state.setText("Not synced")
            self.lbl_map_sync_state.setProperty("class", "unit")
        self.lbl_map_sync_state.style().unpolish(self.lbl_map_sync_state)
        self.lbl_map_sync_state.style().polish(self.lbl_map_sync_state)

    def _on_map_sync_live(self) -> None:
        self._map_sync_active = True
        self._update_map_sync_label()
        self._refresh_map_overlay(pan_map=True)
        if self.controller and self.controller.is_connected():
            if not self.chk_poll.isChecked():
                self.chk_poll.blockSignals(True)
                self.chk_poll.setChecked(True)
                self.chk_poll.blockSignals(False)
                self._toggle_polling(True)
            elif not self._polling_timer.isActive():
                self._polling_timer.start(self.poll_interval.value())
            QTimer.singleShot(0, self._poll_positions)
        self._set_status("Platform synced — live gimbal tracking active")

    def _on_scan_mode_changed(self) -> None:
        if not hasattr(self, "scan_mode"):
            return
        hardware = self.scan_mode.currentData() == "hardware"
        self.scan_loop.setEnabled(not hardware)
        if hardware:
            self.scan_loop.setChecked(False)
        self._update_ui_state()

    def _start_scan(self) -> None:
        if hasattr(self, "scan_mode") and self.scan_mode.currentData() == "hardware":
            self._start_scan_hardware()
        else:
            self._start_scan_demo()

    def _stop_scan(self) -> None:
        if self._hardware_scan_active:
            self._stop_scan_hardware()
        if self._scan_state is not None:
            self._stop_scan_demo()

    def _start_scan_hardware(self) -> None:
        motor = self.motors[0]
        if motor is None:
            QMessageBox.warning(self, "Not connected", "Connect to the controller before starting a hardware scan.")
            return

        if not self._map_sync_active:
            reply = QMessageBox.question(
                self,
                "Sync recommended",
                "Live sync is off. Platform LLA may not match the map.\n"
                "Start hardware scan anyway?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        params = self._calc_scan()
        steps = max(1, int(params["n_passes"]))
        vfov = max(0.5, self.map_vfov.value())
        speed = params["speed"]
        pitch_min = min(self.scan_el_start.value(), self.scan_el_end.value())

        ok = (
            motor.SCN_set_yaw_min_angle(params["az_start"])
            and motor.SCN_set_yaw_max_angle(params["az_end"])
            and motor.SCN_set_pitch_min_angle(pitch_min)
            and motor.SCN_set_num_steps(steps)
            and motor.SCN_set_steps_height(vfov)
            and motor.SCN_set_scan_speed(speed)
            and motor.SCN_start_scan_mode_snake()
        )
        if not ok:
            QMessageBox.critical(self, "Scan failed", "One or more scanner configuration commands failed.")
            return

        self._hardware_scan_params = params
        self._hardware_scan_active = True
        self._scan_elapsed.start()
        if not self.chk_poll.isChecked():
            self.chk_poll.setChecked(True)
        elif not self._polling_timer.isActive():
            self._polling_timer.start(self.poll_interval.value())

        self.scan_progress.setFormat("HARDWARE SCAN %p%")
        self.scan_progress.setValue(0)
        self._run_map_js("if(window.clearScanOverlay) window.clearScanOverlay();")
        self._preview_scan_area()
        self._set_status("Hardware scan started.")
        self._update_ui_state()

    def _stop_scan_hardware(self) -> None:
        motor = self.motors[0]
        if motor is not None:
            motor.SCN_stop_scan()
        self._hardware_scan_active = False
        self._hardware_scan_params = None
        if self._scan_state is None:
            self.scan_progress.setFormat("ABORTED")
        self._set_status("Hardware scan stopped.")
        self._update_ui_state()

    def _update_hardware_scan_progress(self) -> None:
        if not self._hardware_scan_active or self._hardware_scan_params is None:
            return

        yaw = self._axis_positions_last_good[0]
        pitch = self._axis_positions_last_good[1]
        if yaw is None or pitch is None:
            return

        params = self._hardware_scan_params
        elapsed_s = self._scan_elapsed.elapsed() / 1000.0
        progress = min(1.0, elapsed_s / max(0.1, params["total_time"]))
        self.scan_progress.setValue(int(progress * 1000))

        near, center, far = self._compute_fov_distances(float(pitch))
        scan_payload = {
            "az": yaw,
            "el": pitch,
            "boresightHeading": self._computed_map_heading(),
            "boresightDistance": center,
            "centerDistance": center,
            "pass": 0,
            "progress": progress,
            "base_heading": self.map_heading.value(),
            "hfov": self.map_hfov.value(),
            "vfov": self.map_vfov.value(),
            "lat": self.map_lat.value(),
            "lon": self.map_lon.value(),
            "alt": self.map_alt.value(),
            "near": near,
            "far": far,
        }
        self._run_map_js(
            f"if(window.updateScanProgress) window.updateScanProgress({json.dumps(scan_payload)});"
        )

    def _calc_scan(self):
        az_start = self.scan_az_start.value()
        az_end = self.scan_az_end.value()
        el_start = self.scan_el_start.value()
        el_end = self.scan_el_end.value()
        speed = max(0.1, self.scan_speed.value())
        hfov = max(0.5, self.map_hfov.value())
        vfov = max(0.5, self.map_vfov.value())

        az_range = abs(az_end - az_start)
        el_range = abs(el_end - el_start)
        n_passes = max(1, math.ceil(el_range / vfov))
        total_angle = az_range * n_passes + el_range
        total_time = total_angle / speed

        alt = max(1.0, self.map_alt.value())
        avg_pitch = max(1.0, (abs(el_start) + abs(el_end)) / 2.0)
        ground_range = alt / math.tan(math.radians(avg_pitch)) if avg_pitch < 89 else alt
        width_m = 2.0 * ground_range * math.tan(math.radians(az_range / 2.0))
        height_m = 2.0 * ground_range * math.tan(math.radians(el_range / 2.0))
        area_km2 = (width_m * height_m) / 1e6

        self.lbl_scan_info.setText(
            f"AZ range: {az_range:.1f} deg | EL range: {el_range:.1f} deg | "
            f"Passes: {n_passes} | Total travel: {total_angle:.1f} deg"
        )
        mins = int(total_time // 60)
        secs = total_time % 60
        self.lbl_scan_time.setText(
            f"Est. time: {mins}m {secs:.1f}s | "
            f"Ground footprint: {width_m:.0f} x {height_m:.0f} m "
            f"({area_km2:.3f} km\u00b2)"
        )
        return {
            "az_start": min(az_start, az_end),
            "az_end": max(az_start, az_end),
            "el_start": max(el_start, el_end),
            "el_end": min(el_start, el_end),
            "speed": speed,
            "hfov": hfov,
            "vfov": vfov,
            "n_passes": n_passes,
            "total_time": total_time,
        }

    def _start_scan_demo(self):
        params = self._calc_scan()
        self._scan_state = {
            **params,
            "current_az": params["az_start"],
            "current_el": params["el_start"],
            "pass_idx": 0,
            "direction": 1,
            "scanned_polygons": [],
        }
        self._scan_elapsed.start()
        self._scan_timer.start()
        self.scan_progress.setFormat("SCANNING %p%")
        self._run_map_js("if(window.clearScanOverlay) window.clearScanOverlay();")
        self._preview_scan_area()
        self._set_status("Scan demo running...")
        self._update_ui_state()

    def _stop_scan_demo(self):
        self._scan_timer.stop()
        self._scan_state = None
        if not self._hardware_scan_active:
            self.scan_progress.setFormat("ABORTED")
        self._set_status("Scan demo stopped.")
        self._update_ui_state()

    def _tick_scan_demo(self):
        s = self._scan_state
        if s is None:
            self._stop_scan_demo()
            return

        dt = 0.05
        step_angle = s["speed"] * dt
        s["current_az"] += step_angle * s["direction"]

        sweep_done = False
        if s["direction"] == 1 and s["current_az"] >= s["az_end"]:
            s["current_az"] = s["az_end"]
            sweep_done = True
        elif s["direction"] == -1 and s["current_az"] <= s["az_start"]:
            s["current_az"] = s["az_start"]
            sweep_done = True

        if sweep_done:
            s["pass_idx"] += 1
            if s["pass_idx"] >= s["n_passes"]:
                if self.scan_loop.isChecked():
                    s["pass_idx"] = 0
                    s["direction"] = 1
                    s["current_az"] = s["az_start"]
                    s["current_el"] = s["el_start"]
                    self._scan_elapsed.restart()
                    self.scan_progress.setFormat("LOOP SCAN %p%")
                    self._run_map_js("if(window.clearScanOverlay) window.clearScanOverlay();")
                    self._set_status("Scan loop restarting...")
                else:
                    self._scan_timer.stop()
                    self.scan_progress.setValue(1000)
                    self.scan_progress.setFormat("COMPLETE")
                    self._scan_state = None
                    self._set_status("Scan complete.")
                    self._update_ui_state()
                return
            s["direction"] *= -1
            s["current_el"] -= s["vfov"]
            if s["current_el"] < s["el_end"]:
                s["current_el"] = s["el_end"]

        elapsed_s = self._scan_elapsed.elapsed() / 1000.0
        progress = min(1.0, elapsed_s / max(0.1, s["total_time"]))
        self.scan_progress.setValue(int(progress * 1000))

        heading = self.map_heading.value() + s["current_az"]
        self._axis_positions[0] = s["current_az"]
        self._axis_positions[1] = s["current_el"]
        self._axis_positions_last_good[0] = s["current_az"]
        self._axis_positions_last_good[1] = s["current_el"]
        self._schedule_map_overlay_refresh()

        near, center, far = self._compute_fov_distances(float(s["current_el"]))
        scan_payload = {
            "az": s["current_az"],
            "el": s["current_el"],
            "boresightHeading": self._computed_map_heading(),
            "boresightDistance": center,
            "centerDistance": center,
            "pass": s["pass_idx"],
            "progress": progress,
            "base_heading": self.map_heading.value(),
            "hfov": self.map_hfov.value(),
            "vfov": self.map_vfov.value(),
            "lat": self.map_lat.value(),
            "lon": self.map_lon.value(),
            "alt": self.map_alt.value(),
            "near": near,
            "far": far,
        }
        self._run_map_js(f"if(window.updateScanProgress) window.updateScanProgress({json.dumps(scan_payload)});")

    def _scan_params_payload(self) -> dict:
        p = self._calc_scan()
        base_hdg = self.map_heading.value()
        pitch = self._axis_positions_last_good[1]
        near, _center, far = self._compute_fov_distances(pitch)
        return {
            "az_start": p["az_start"],
            "az_end": p["az_end"],
            "el_start": p["el_start"],
            "el_end": p["el_end"],
            "base_heading": base_hdg,
            "hfov": p["hfov"],
            "vfov": p["vfov"],
            "lat": self.map_lat.value(),
            "lon": self.map_lon.value(),
            "alt": self.map_alt.value(),
            "near": near,
            "far": far,
            "n_passes": p["n_passes"],
        }

    def _preview_scan_area(self):
        payload = self._scan_params_payload()
        self._run_map_js(f"if(window.showScanAreaPreview) window.showScanAreaPreview({json.dumps(payload)});")
        self._set_status("Scan area preview shown on map.")

    def _on_map_legend_toggled(self, checked: bool):
        self._run_map_js(f"if(window.setLegendVisible) window.setLegendVisible({str(bool(checked)).lower()});")

    def _on_map_fill_toggled(self, checked: bool):
        self._run_map_js(f"if(window.setFovFillVisible) window.setFovFillVisible({str(bool(checked)).lower()});")
        self._schedule_map_overlay_refresh()

    def _on_cesium_view_mode_changed(self):
        if self.map_engine.currentData() != "cesium":
            return
        self._apply_cesium_camera()

    def _apply_cesium_camera(self):
        if self.map_engine.currentData() != "cesium" or not self._map_loaded:
            return
        mode = self.cesium_view_mode.currentData()
        payload = {
            "mode": mode,
            "bearing": self.cesium_orbit_bearing.value(),
            "pitch": self.cesium_orbit_pitch.value(),
            "range": self.cesium_orbit_range.value(),
        }
        self._run_map_js(f"if(window.setCameraView) window.setCameraView({json.dumps(payload)});")

    def _nudge_cesium_camera(self, d_bearing: float, d_pitch: float, d_range: float):
        if self.map_engine.currentData() != "cesium":
            return
        self.cesium_orbit_bearing.setValue(self.cesium_orbit_bearing.value() + d_bearing)
        self.cesium_orbit_pitch.setValue(
            max(-89.0, min(0.0, self.cesium_orbit_pitch.value() + d_pitch))
        )
        self.cesium_orbit_range.setValue(
            max(50.0, self.cesium_orbit_range.value() + d_range)
        )
        self._apply_cesium_camera()

    def _send_map_fov_to_controller(self):
        motor = self.motors[0]
        if motor is None:
            self._set_status("Connect to controller before sending camera FOV.")
            return

        fov_h = self.map_hfov.value()
        fov_v = self.map_vfov.value()
        ok = motor.set_camera_fov((fov_h, fov_v))
        if ok:
            self._set_status(f"Camera FOV sent: horizontal={fov_h:.1f}, vertical={fov_v:.1f}.")
        else:
            self._set_status("Failed to send camera FOV.")

    # =====================================================================
    #  Connection actions
    # =====================================================================
    def _on_connect(self):
        host = self.host_input.text().strip()
        port = self.port_input.value()
        if not host:
            QMessageBox.warning(self, "Input error", "Please enter a valid host address.")
            return

        self._set_status(f"Connecting to {host}:{port}...")
        QApplication.processEvents()

        ctrl = Controller(host=host, port=port, timeout=3.0)
        ok = ctrl.connect()
        if ok:
            self.controller = ctrl
            self.motors[0] = MotorControl(ctrl, axis_number=1)
            self.motors[1] = MotorControl(ctrl, axis_number=2)
            self.conn_indicator.set_color("#a6e3a1")
            self._set_status(f"Connected to {host}:{port}")
            self._schedule_map_overlay_refresh()
            self._update_map_live_status()
        else:
            self.controller = None
            self.motors = [None, None]
            self.conn_indicator.set_color("#f38ba8")
            self._set_status(f"Connection failed  ({host}:{port})")
            QMessageBox.critical(self, "Connection Error",
                                 f"Could not connect to {host}:{port}.\n"
                                 "Check that the controller is powered on and reachable.")
        self._update_ui_state()

    def _on_disconnect(self):
        self.chk_poll.blockSignals(True)
        self.chk_poll.setChecked(False)
        self.chk_poll.blockSignals(False)
        if self._hardware_scan_active:
            self._stop_scan_hardware()
        self._stop_scan_demo()
        self._map_sync_active = False
        self._update_map_sync_label()
        self._wait_for_io_threads()
        if self.controller:
            self.controller.disconnect()
        self.controller = None
        self.motors = [None, None]
        self._axis_positions = [None, None]
        self._axis_positions_last_good = [None, None]
        self.conn_indicator.set_color("#585b70")
        self._set_status("Disconnected")
        self._refresh_map_overlay(pan_map=False)
        self._update_map_live_status()
        self._update_ui_state()

    # =====================================================================
    #  Generic motor command dispatcher
    # =====================================================================
    def _cmd(self, axis_idx: int, method_name: str, *args):
        motor = self.motors[axis_idx]
        if motor is None:
            self._set_status("Not connected")
            return
        if self._io_busy():
            self._set_status("Busy — wait for prior command to finish")
            return
        func = getattr(motor, method_name)
        desc = f"Axis {axis_idx + 1}: {method_name}"
        read_after = (
            method_name in self._POST_MOVE_READ_METHODS
            and self._map_sync_active
        )
        worker = MotorWorker(
            func,
            *args,
            description=desc,
            axis_index=axis_idx,
            motors=list(self.motors),
            read_positions_after=read_after,
        )
        worker.command_finished.connect(
            lambda d, ok, i=axis_idx: self._on_cmd_finished(d, ok, i)
        )
        if not self._register_worker(worker):
            self._set_status("Busy — wait for prior command to finish")
            return
        worker.start()
        self._set_status(f"Sending {desc}...")

    def _on_cmd_finished(self, desc: str, success: bool, axis_idx: int = 0):
        tag = "OK" if success else "FAIL"
        self._set_status(f"[{tag}] {desc}")
        if success and self._map_sync_active:
            self._update_axis_display_labels()
            self._schedule_map_overlay_refresh()
            if "SSL" in desc:
                QTimer.singleShot(120, self._poll_positions)

    # =====================================================================
    #  Position / speed read-back
    # =====================================================================
    def _read_position(self, axis_idx: int):
        motor = self.motors[axis_idx]
        if motor is None or self._io_busy():
            return
        worker = MotorWorker(
            motor.get_position,
            description="read_position",
            axis_index=axis_idx,
            read_type="position",
        )
        worker.position_read.connect(self._on_position_read)
        if not self._register_worker(worker):
            return
        worker.start()

    def _read_speed(self, axis_idx: int):
        if self._map_sync_active:
            return
        motor = self.motors[axis_idx]
        if motor is None or self._io_busy():
            return
        worker = MotorWorker(
            motor.get_speed,
            description="read_speed",
            axis_index=axis_idx,
            read_type="speed",
        )
        worker.speed_read.connect(self._on_speed_read)
        if not self._register_worker(worker):
            return
        worker.start()

    def _on_position_read(self, axis_idx: int, value):
        self._accept_axis_position(axis_idx, value)
        self._update_axis_display_labels()
        self._schedule_map_overlay_refresh()
        if self._hardware_scan_active:
            self._update_hardware_scan_progress()

    def _on_speed_read(self, axis_idx: int, value):
        lbl = self.axis_panels[axis_idx]["lbl_spd"]
        if value is not None:
            lbl.setText(f"{value:.2f}")
        else:
            lbl.setText("err")

    # =====================================================================
    #  Polling
    # =====================================================================
    def _toggle_polling(self, on: bool):
        if on and self.controller and self.controller.is_connected():
            self._polling_timer.start(self.poll_interval.value())
        else:
            self._polling_timer.stop()
        self._update_map_live_status()

    def _poll_positions(self):
        if self.controller is None or not self.controller.is_connected():
            return
        if self._io_busy():
            return

        self._poll_worker = PositionPollWorker(list(self.motors))
        self._poll_worker.position_read.connect(self._on_position_read)
        self._poll_worker.poll_data_ready.connect(
            self._on_poll_data_ready, Qt.QueuedConnection
        )
        self._register_worker(self._poll_worker)
        self._poll_worker.start()

    def _on_poll_data_ready(self) -> None:
        if not self._map_sync_active:
            self._speed_poll_counter += 1
            if self._speed_poll_counter >= 8:
                self._speed_poll_counter = 0
                self._read_speed(0)

    # =====================================================================
    #  SSL commands
    # =====================================================================
    def _on_ssl_position(self):
        motor = self.motors[0]
        if motor is None:
            self._set_status("Not connected")
            return
        mode = self.ssl_pos_mode.currentIndex()
        p1 = self.ssl_pos1.value()
        p2 = self.ssl_pos2.value()
        desc = f"SSL position ({p1}, {p2}, mode={mode})"
        worker = MotorWorker(motor.SSL_position, p1, p2, mode, description=desc)
        worker.command_finished.connect(
            lambda d, ok: self._on_cmd_finished(d, ok, 0)
        )
        self._register_worker(worker)
        worker.start()
        self._set_status(f"Sending {desc}...")

    def _on_ssl_speed(self):
        motor = self.motors[0]
        if motor is None:
            self._set_status("Not connected")
            return
        s1 = self.ssl_spd1.value()
        s2 = self.ssl_spd2.value()
        desc = f"SSL speed ({s1}, {s2})"
        worker = MotorWorker(motor.SSL_speed, s1, s2, description=desc)
        worker.command_finished.connect(
            lambda d, ok: self._on_cmd_finished(d, ok, 0)
        )
        self._register_worker(worker)
        worker.start()
        self._set_status(f"Sending {desc}...")

    # =====================================================================
    #  Configuration
    # =====================================================================
    def _apply_config(self, axis_idx: int):
        motor = self.motors[axis_idx]
        cfg = self.cfg_widgets[axis_idx]
        new_max_pos = cfg["max_pos"].value()
        new_max_spd = cfg["max_spd"].value()
        new_max_acc = cfg["max_acc"].value()

        if motor is not None:
            motor.max_position = new_max_pos
            motor.max_speed = new_max_spd
            motor.max_acceleration = new_max_acc

        # Also update the spin-box ranges in the motor tab
        panel = self.axis_panels[axis_idx]
        panel["spin_pos"].setRange(-new_max_pos, new_max_pos)
        panel["spin_spd"].setRange(-new_max_spd, new_max_spd)
        panel["spin_acc"].setRange(0, new_max_acc)

        self._set_status(
            f"Axis {axis_idx + 1} config applied  "
            f"(pos ±{new_max_pos}, spd ±{new_max_spd}, acc {new_max_acc})"
        )

    # =====================================================================
    #  UI helpers
    # =====================================================================
    def _update_ui_state(self):
        """Enable/disable widgets based on connection state."""
        connected = self.controller is not None and self.controller.is_connected()
        self.btn_connect.setEnabled(not connected)
        self.host_input.setEnabled(not connected)
        self.port_input.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)

        # Motor tabs
        self.tabs.setEnabled(True)
        for panel in self.axis_panels:
            for key, widget in panel.items():
                if key == "group":
                    continue
                if isinstance(widget, (QPushButton, QDoubleSpinBox, QComboBox)):
                    widget.setEnabled(connected)

        if hasattr(self, "btn_send_fov_api"):
            self.btn_send_fov_api.setEnabled(connected)
        if hasattr(self, "btn_map_read_axes"):
            self.btn_map_read_axes.setEnabled(connected)
        if hasattr(self, "btn_map_sync_live"):
            self.btn_map_sync_live.setEnabled(True)
        if hasattr(self, "btn_start_scan"):
            scan_running = self._hardware_scan_active or self._scan_state is not None
            if scan_running:
                self.btn_start_scan.setEnabled(False)
                self.btn_stop_scan.setEnabled(True)
            else:
                hardware_mode = (
                    hasattr(self, "scan_mode")
                    and self.scan_mode.currentData() == "hardware"
                )
                self.btn_start_scan.setEnabled(connected if hardware_mode else True)
                self.btn_stop_scan.setEnabled(False)
        self._update_map_live_status()

    def _set_status(self, msg: str):
        self.status_bar.showMessage(msg)

    def closeEvent(self, event):
        self._map_overlay_timer.stop()
        self._scan_timer.stop()
        if self._hardware_scan_active:
            self._stop_scan_hardware()
        self._wait_for_io_threads(4000)
        self._map_reload_timer.stop()
        if hasattr(self, "_map_health_timer"):
            self._map_health_timer.stop()
        if self._map_tmp_file:
            try:
                os.unlink(self._map_tmp_file)
            except OSError:
                pass
        if self.controller:
            self.controller.disconnect()
        event.accept()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    app.setStyleSheet(DARK_STYLESHEET)
    window = GimbalControlGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
