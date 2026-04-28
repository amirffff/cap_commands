"""
Gimbal Control GUI - PyQt5 Application
Controls a 2-axis gimbal via the MotorControl/Controller classes.
"""

import sys
import struct
from functools import partial
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QDoubleSpinBox, QComboBox,
    QStatusBar, QTabWidget, QGridLayout, QFrame, QSplitter,
    QMessageBox, QCheckBox, QSpinBox, QLineEdit
)
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon

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
QLabel[class="unit"] {
    color: #6c7086;
    font-size: 11px;
}
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
    finished = pyqtSignal(str, bool)       # (description, success)
    position_read = pyqtSignal(int, object)  # (axis_index, value_or_None)
    speed_read = pyqtSignal(int, object)

    def __init__(self, func, *args, description="", axis_index=0, read_type=""):
        super().__init__()
        self._func = func
        self._args = args
        self._desc = description
        self._axis = axis_index
        self._read_type = read_type

    def run(self):
        try:
            result = self._func(*self._args)
            if self._read_type == "position":
                self.position_read.emit(self._axis, result)
            elif self._read_type == "speed":
                self.speed_read.emit(self._axis, result)
            else:
                self.finished.emit(self._desc, bool(result))
        except Exception as e:
            self.finished.emit(f"{self._desc} - Error: {e}", False)


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
        self.setMinimumSize(1020, 720)
        self.resize(1100, 780)

        # ---- State ----------------------------------------------------------
        self.controller: Controller | None = None
        self.motors: list[MotorControl | None] = [None, None]
        self._workers: list[MotorWorker] = []
        self._polling_timer = QTimer(self)
        self._polling_timer.timeout.connect(self._poll_positions)

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
        self.tabs.addTab(self._build_motor_tab(), "Motor Control")
        self.tabs.addTab(self._build_ssl_tab(), "SSL (Sync Both)")
        self.tabs.addTab(self._build_config_tab(), "Configuration")
        root_layout.addWidget(self.tabs)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self._set_status("Disconnected")

        self._update_ui_state()

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
        self.chk_poll = QCheckBox("Auto-read positions")
        self.chk_poll.setChecked(False)
        self.chk_poll.toggled.connect(self._toggle_polling)
        lay.addWidget(self.chk_poll)

        self.poll_interval = QSpinBox()
        self.poll_interval.setRange(30, 10000)
        self.poll_interval.setValue(30)
        self.poll_interval.setSuffix(" ms")
        self.poll_interval.setFixedWidth(100)
        self.poll_interval.valueChanged.connect(
            lambda v: self._polling_timer.setInterval(v) if self._polling_timer.isActive() else None
        )
        lay.addWidget(self.poll_interval)

        return grp

    # =====================================================================
    # Motor Control tab – one panel per axis
    # =====================================================================
    def _build_motor_tab(self) -> QWidget:
        page = QWidget()
        lay = QHBoxLayout(page)
        lay.setSpacing(10)

        self.axis_panels: list[dict] = []
        for idx in range(2):
            panel = self._build_axis_panel(idx)
            lay.addWidget(panel["group"])
            self.axis_panels.append(panel)
        return page

    def _build_axis_panel(self, idx: int) -> dict:
        axis_num = idx + 1
        grp = QGroupBox(f"Axis {axis_num}  (Motor {axis_num})")
        grid = QGridLayout(grp)
        grid.setSpacing(8)
        row = 0
        widgets: dict = {"group": grp}

        # -- Enable / Disable / Reset / Homing --------------------------------
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

        for c, btn in enumerate([btn_on, btn_off, btn_reset, btn_home]):
            grid.addWidget(btn, row, c)
        widgets["btn_on"] = btn_on
        widgets["btn_off"] = btn_off
        row += 1

        # -- Separator ---------------------------------------------------------
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setProperty("class", "separator")
        grid.addWidget(sep, row, 0, 1, 4)
        row += 1

        # -- Movement mode & type ----------------------------------------------
        grid.addWidget(QLabel("Mode:"), row, 0)
        combo_mode = QComboBox()
        combo_mode.addItems(["position", "speed"])
        grid.addWidget(combo_mode, row, 1)
        btn_set_mode = QPushButton("Set Mode")
        btn_set_mode.clicked.connect(
            lambda checked, i=idx, cb=combo_mode: self._cmd(i, "set_movement_mode", cb.currentText())
        )
        grid.addWidget(btn_set_mode, row, 2)
        widgets["combo_mode"] = combo_mode
        row += 1

        grid.addWidget(QLabel("Type:"), row, 0)
        combo_type = QComboBox()
        combo_type.addItems(["absolute", "relative"])
        grid.addWidget(combo_type, row, 1)
        btn_set_type = QPushButton("Set Type")
        btn_set_type.clicked.connect(
            lambda checked, i=idx, cb=combo_type: self._cmd(i, "set_movement_type", cb.currentText())
        )
        grid.addWidget(btn_set_type, row, 2)
        widgets["combo_type"] = combo_type
        row += 1

        # -- Separator ---------------------------------------------------------
        sep2 = QFrame()
        sep2.setFrameShape(QFrame.HLine)
        sep2.setProperty("class", "separator")
        grid.addWidget(sep2, row, 0, 1, 4)
        row += 1

        # -- Position ----------------------------------------------------------
        grid.addWidget(QLabel("Position:"), row, 0)
        spin_pos = QDoubleSpinBox()
        spin_pos.setRange(-9999.0, 9999.0)
        spin_pos.setDecimals(2)
        spin_pos.setSuffix("  deg")
        grid.addWidget(spin_pos, row, 1, 1, 2)
        btn_go_pos = QPushButton("Go To Position")
        btn_go_pos.setProperty("class", "accent")
        btn_go_pos.clicked.connect(
            lambda checked, i=idx, sp=spin_pos: self._cmd(i, "set_position", sp.value())
        )
        grid.addWidget(btn_go_pos, row, 3)
        widgets["spin_pos"] = spin_pos
        row += 1

        # -- Speed -------------------------------------------------------------
        grid.addWidget(QLabel("Speed:"), row, 0)
        spin_spd = QDoubleSpinBox()
        spin_spd.setRange(-9999.0, 9999.0)
        spin_spd.setDecimals(2)
        spin_spd.setSuffix("  deg/s")
        grid.addWidget(spin_spd, row, 1, 1, 2)
        btn_set_spd = QPushButton("Set Speed")
        btn_set_spd.setProperty("class", "accent")
        btn_set_spd.clicked.connect(
            lambda checked, i=idx, sp=spin_spd: self._cmd(i, "set_speed", sp.value())
        )
        grid.addWidget(btn_set_spd, row, 3)
        widgets["spin_spd"] = spin_spd
        row += 1

        # -- Acceleration ------------------------------------------------------
        grid.addWidget(QLabel("Acceleration:"), row, 0)
        spin_acc = QDoubleSpinBox()
        spin_acc.setRange(0.0, 9999.0)
        spin_acc.setDecimals(2)
        spin_acc.setSuffix("  deg/s²")
        spin_acc.setValue(10.0)
        grid.addWidget(spin_acc, row, 1, 1, 2)
        btn_set_acc = QPushButton("Set Acceleration")
        btn_set_acc.clicked.connect(
            lambda checked, i=idx, sp=spin_acc: self._cmd(i, "set_acceleration", sp.value())
        )
        grid.addWidget(btn_set_acc, row, 3)
        widgets["spin_acc"] = spin_acc
        row += 1

        # -- Update button (sends update command) ------------------------------
        btn_update = QPushButton("Update (execute)")
        btn_update.setProperty("class", "accent")
        btn_update.clicked.connect(lambda checked, i=idx: self._cmd(i, "update"))
        grid.addWidget(btn_update, row, 0, 1, 4)
        row += 1

        # -- Separator ---------------------------------------------------------
        sep3 = QFrame()
        sep3.setFrameShape(QFrame.HLine)
        sep3.setProperty("class", "separator")
        grid.addWidget(sep3, row, 0, 1, 4)
        row += 1

        # -- Readouts ----------------------------------------------------------
        grid.addWidget(QLabel("Current Position:"), row, 0, 1, 2)
        lbl_pos = QLabel("---")
        lbl_pos.setProperty("class", "value")
        grid.addWidget(lbl_pos, row, 2)
        btn_read_pos = QPushButton("Read")
        btn_read_pos.clicked.connect(partial(self._read_position, idx))
        grid.addWidget(btn_read_pos, row, 3)
        widgets["lbl_pos"] = lbl_pos
        row += 1

        grid.addWidget(QLabel("Current Speed:"), row, 0, 1, 2)
        lbl_spd = QLabel("---")
        lbl_spd.setProperty("class", "value")
        grid.addWidget(lbl_spd, row, 2)
        btn_read_spd = QPushButton("Read")
        btn_read_spd.clicked.connect(partial(self._read_speed, idx))
        grid.addWidget(btn_read_spd, row, 3)
        widgets["lbl_spd"] = lbl_spd
        row += 1

        grid.setRowStretch(row, 1)
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
        self._polling_timer.stop()
        self.chk_poll.setChecked(False)
        if self.controller:
            self.controller.disconnect()
        self.controller = None
        self.motors = [None, None]
        self.conn_indicator.set_color("#585b70")
        self._set_status("Disconnected")
        self._update_ui_state()

    # =====================================================================
    #  Generic motor command dispatcher
    # =====================================================================
    def _cmd(self, axis_idx: int, method_name: str, *args):
        motor = self.motors[axis_idx]
        if motor is None:
            self._set_status("Not connected")
            return
        func = getattr(motor, method_name)
        desc = f"Axis {axis_idx + 1}: {method_name}"
        worker = MotorWorker(func, *args, description=desc, axis_index=axis_idx)
        worker.finished.connect(self._on_cmd_finished)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker) if worker in self._workers else None)
        worker.start()
        self._set_status(f"Sending {desc}...")

    def _on_cmd_finished(self, desc: str, success: bool):
        tag = "OK" if success else "FAIL"
        self._set_status(f"[{tag}] {desc}")

    # =====================================================================
    #  Position / speed read-back
    # =====================================================================
    def _read_position(self, axis_idx: int):
        motor = self.motors[axis_idx]
        if motor is None:
            return
        worker = MotorWorker(motor.get_position, description="read_position",
                             axis_index=axis_idx, read_type="position")
        worker.position_read.connect(self._on_position_read)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker) if worker in self._workers else None)
        worker.start()

    def _read_speed(self, axis_idx: int):
        motor = self.motors[axis_idx]
        if motor is None:
            return
        worker = MotorWorker(motor.get_speed, description="read_speed",
                             axis_index=axis_idx, read_type="speed")
        worker.speed_read.connect(self._on_speed_read)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker) if worker in self._workers else None)
        worker.start()

    def _on_position_read(self, axis_idx: int, value):
        lbl = self.axis_panels[axis_idx]["lbl_pos"]
        if value is not None:
            lbl.setText(f"{value:.2f}")
        else:
            lbl.setText("err")

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

    def _poll_positions(self):
        for idx in range(2):
            self._read_position(idx)
            self._read_speed(idx)

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
        worker.finished.connect(self._on_cmd_finished)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker) if worker in self._workers else None)
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
        worker.finished.connect(self._on_cmd_finished)
        self._workers.append(worker)
        worker.finished.connect(lambda: self._workers.remove(worker) if worker in self._workers else None)
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

    def _set_status(self, msg: str):
        self.status_bar.showMessage(msg)

    def closeEvent(self, event):
        self._polling_timer.stop()
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
