"""
Captrack Motor Configuration Application

Qt-based GUI for configuring motor controller systems via TCP/IP.
Supports 1-4 axes (auto-detected from system or JSON).
Parameters separated by motor_driver.py sections.
Wireshark-style packet log with raw hex data.
"""

import sys
import json
import os
import time
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox, QSpinBox,
    QDoubleSpinBox, QFileDialog, QTabWidget, QGridLayout,
    QMessageBox, QFrame, QScrollArea, QSplitter, QTreeWidget,
    QTreeWidgetItem, QCheckBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont, QColor

from motor_driver import Controller, MotorControl


UNKNOWN = "UNKNOWN"
MAX_RETRIES = 2
RETRY_DELAY_MS = 80
CFG_LOAD_DELAY = 1

PARAM_DESCRIPTIONS = {
    "motor_type": "Motor type: Stepper / BLDC / DC",
    "gear_ratio": "Gear ratio motor-to-load",
    "max_vdc": "Max DC voltage (V)",
    "peak_current": "Peak current limit (A)",
    "slow_loop_sampling": "Position loop period (μs)",
    "micro_steps": "Micro-step divisions (stepper)",
    "steps_per_rev": "Steps per full revolution",
    "close_loop_enable": "Closed-loop feedback on/off",
    "encoder_lines": "Motor encoder resolution (lines)",
    "load_encoder_lines": "Load encoder resolution (lines)",
    "encoder_location": "Encoder physical location",
    "apos_load_type": "Absolute position source type",
    "biss_resolution": "BiSS encoder bits",
    "biss_com": "BiSS SPI port",
    "abs_enc_offset": "Abs encoder zero offset (°)",
    "abs_enc_reverse": "Abs encoder direction (0/1)",
    "max_speed": "Max speed limit",
    "min_speed": "Min speed threshold",
    "max_acceleration": "Max acceleration limit",
    "min_acceleration": "Min acceleration limit",
    "pc_com": "PC host interface type",
    "ts_com_type": "Tracking system comm type",
    "can_baud_rate": "CAN bus baud rate",
    "com_axes_ts": "Axis ID on TS bus",
    "multi_com": "Active communication channels (bitmask)",
    "reversed_imu": "IMU sensor assignment",
    "imu_com": "IMU serial port",
    "imu_baud_rate": "IMU baud rate (bps)",
    "imu_freq": "IMU output frequency (Hz)",
    "system_type": "System operating mode",
    "firmware_version": "Firmware version (read-only)",
    "serial_number": "Controller serial number",
    "controller_ip": "Controller IP address",
    "controller_port": "Controller TCP port",
    "controller_subnet": "Controller subnet mask",
}

STYLESHEET = """
* {
    font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif;
    font-size: 10pt;
}
QMainWindow {
    background-color: #0f1318;
}
QGroupBox {
    font-size: 13pt;
    font-weight: bold;
    border: 2px solid #2563eb;
    border-radius: 8px;
    margin-top: 16px;
    padding: 18px 10px 10px 10px;
    color: #93c5fd;
    background-color: #141a24;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 3px 12px;
    background-color: #0f1318;
    border: 1px solid #2563eb;
    border-radius: 4px;
    font-size: 12pt;
}
QLabel {
    color: #d1d5db;
    font-size: 10pt;
}
QLabel#param_label {
    color: #e2e8f0;
    font-size: 10pt;
    font-weight: bold;
}
QLabel#desc_label {
    color: #6b7b8d;
    font-size: 8pt;
    font-style: italic;
}
QLabel#status_ok {
    color: #34d399;
    font-size: 9pt;
    font-weight: bold;
}
QLabel#status_err {
    color: #f87171;
    font-size: 9pt;
    font-weight: bold;
}
QLabel#status_loaded {
    color: #60a5fa;
    font-size: 9pt;
    font-weight: bold;
}
QLabel#info_label {
    color: #94a3b8;
    font-size: 9pt;
}
QLabel#info_value {
    color: #e2e8f0;
    font-size: 9pt;
    font-weight: bold;
}
QLineEdit, QSpinBox, QDoubleSpinBox {
    background-color: #1e293b;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 4px 8px;
    color: #f1f5f9;
    font-size: 10pt;
    min-height: 28px;
}
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus {
    border: 2px solid #3b82f6;
}
QLineEdit:read-only {
    background-color: #0f172a;
    color: #94a3b8;
    border-color: #1e293b;
}
QComboBox {
    background-color: #1e293b;
    border: 1px solid #334155;
    border-radius: 5px;
    padding: 4px 8px;
    color: #f1f5f9;
    font-size: 10pt;
    min-height: 28px;
    min-width: 110px;
}
QComboBox:focus, QComboBox:on {
    border: 2px solid #3b82f6;
}
QComboBox::drop-down {
    border: none;
    width: 26px;
    subcontrol-position: center right;
}
QComboBox::down-arrow {
    image: none;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 7px solid #94a3b8;
    margin-right: 8px;
}
QComboBox QAbstractItemView {
    background-color: #1e293b;
    border: 2px solid #3b82f6;
    border-radius: 4px;
    color: #f1f5f9;
    selection-background-color: #2563eb;
    selection-color: #ffffff;
    font-size: 10pt;
    padding: 4px;
    outline: none;
}
QComboBox QAbstractItemView::item {
    min-height: 28px;
    padding: 4px 8px;
}
QComboBox QAbstractItemView::item:hover {
    background-color: #334155;
}
QPushButton {
    background-color: #1e293b;
    border: 1px solid #475569;
    border-radius: 5px;
    padding: 6px 14px;
    color: #e2e8f0;
    font-size: 9pt;
    font-weight: bold;
    min-height: 28px;
}
QPushButton:hover {
    background-color: #334155;
    border-color: #64748b;
}
QPushButton:pressed {
    background-color: #0f172a;
}
QPushButton:disabled {
    background-color: #0f172a;
    color: #475569;
    border-color: #1e293b;
}
QPushButton#btn_get {
    background-color: #064e3b;
    border-color: #059669;
    color: #6ee7b7;
    min-width: 46px;
    max-width: 46px;
    font-size: 9pt;
}
QPushButton#btn_get:hover {
    background-color: #065f46;
    border-color: #34d399;
}
QPushButton#btn_set {
    background-color: #7c2d12;
    border-color: #ea580c;
    color: #fdba74;
    min-width: 46px;
    max-width: 46px;
    font-size: 9pt;
}
QPushButton#btn_set:hover {
    background-color: #9a3412;
    border-color: #fb923c;
}
QPushButton#btn_connect {
    background-color: #064e3b;
    border-color: #10b981;
    color: #6ee7b7;
    font-size: 10pt;
    padding: 6px 20px;
}
QPushButton#btn_connect:hover {
    background-color: #065f46;
}
QPushButton#btn_disconnect {
    background-color: #7f1d1d;
    border-color: #ef4444;
    color: #fca5a5;
    font-size: 10pt;
    padding: 6px 20px;
}
QPushButton#btn_disconnect:hover {
    background-color: #991b1b;
}
QTabWidget::pane {
    border: 2px solid #1e293b;
    background-color: #0f1318;
    border-radius: 6px;
}
QTabBar::tab {
    background-color: #1e293b;
    border: 1px solid #334155;
    padding: 9px 22px;
    color: #94a3b8;
    font-size: 10pt;
    font-weight: bold;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
    margin-right: 3px;
}
QTabBar::tab:selected {
    background-color: #2563eb;
    color: #ffffff;
    border-color: #2563eb;
}
QTabBar::tab:hover:!selected {
    background-color: #334155;
    color: #e2e8f0;
}
QTreeWidget {
    background-color: #0a0e14;
    border: 1px solid #1e293b;
    color: #d1d5db;
    font-family: "Cascadia Code", "JetBrains Mono", "Consolas", monospace;
    font-size: 9pt;
    border-radius: 6px;
}
QTreeWidget::item {
    padding: 3px 4px;
    border-bottom: 1px solid #111827;
}
QTreeWidget::item:alternate {
    background-color: #0d1420;
}
QTreeWidget QHeaderView::section {
    background-color: #111827;
    border: none;
    border-bottom: 2px solid #2563eb;
    padding: 7px 6px;
    color: #93c5fd;
    font-weight: bold;
    font-size: 9pt;
}
QScrollArea {
    border: none;
    background-color: #0f1318;
}
QScrollBar:vertical {
    background: #0f1318;
    width: 10px;
    border-radius: 5px;
}
QScrollBar::handle:vertical {
    background: #334155;
    border-radius: 5px;
    min-height: 30px;
}
QScrollBar::handle:vertical:hover {
    background: #475569;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
QStatusBar {
    background-color: #111827;
    color: #94a3b8;
    border-top: 1px solid #1e293b;
    font-size: 9pt;
    padding: 2px 8px;
}
QCheckBox {
    color: #d1d5db;
    spacing: 6px;
    font-size: 9pt;
}
QCheckBox::indicator {
    width: 16px;
    height: 16px;
    border: 1px solid #475569;
    border-radius: 3px;
    background-color: #1e293b;
}
QCheckBox::indicator:checked {
    background-color: #2563eb;
    border-color: #3b82f6;
}
"""


class ConfigApp(QMainWindow):
    """Main configuration application window."""

    MOTOR_TYPES = {0: 'Stepper', 1: 'BLDC', 2: 'DC'}
    MOTOR_TYPES_REV = {'Stepper': 0, 'BLDC': 1, 'DC': 2}

    SYSTEM_TYPES = {0: 'manual', 1: 'stabilized', 2: 'tracker', 3: 'dual_gimbal'}
    SYSTEM_TYPES_REV = {'manual': 0, 'stabilized': 1, 'tracker': 2, 'dual_gimbal': 3}

    ENCODER_LOCATIONS = {0: 'None', 1: 'Motor', 2: 'Load'}
    ENCODER_LOCATIONS_REV = {'None': 0, 'Motor': 1, 'Load': 2}

    APOS_LOAD_TYPES = {0: 'Apos_load', 1: 'Apos_SSI', 2: 'TPOS'}
    APOS_LOAD_TYPES_REV = {'Apos_load': 0, 'Apos_SSI': 1, 'TPOS': 2}

    CLOSE_LOOP = {0: 'Disabled', 1: 'Enabled'}
    CLOSE_LOOP_REV = {'Disabled': 0, 'Enabled': 1}

    PC_COM_TYPES = {0: 'None', 1: 'Ethernet', 2: 'RS232', 3: 'RS422', 4: 'RS485', 5: 'TTL', 6: 'SPI'}
    PC_COM_TYPES_REV = {'None': 0, 'Ethernet': 1, 'RS232': 2, 'RS422': 3, 'RS485': 4, 'TTL': 5, 'SPI': 6}

    TS_COM_TYPES = {0: 'Ethernet', 1: 'RS232', 9: 'CAN'}
    TS_COM_TYPES_REV = {'Ethernet': 0, 'RS232': 1, 'CAN': 9}

    IMU_REVERSED = {0: 'client', 1: 'base_imu', 2: 'load_imu', 3: 'mid_imu'}
    IMU_REVERSED_REV = {'client': 0, 'base_imu': 1, 'load_imu': 2, 'mid_imu': 3}

    IMU_COM_TYPES = {0: 'None', 1: 'RS232_1', 2: 'RS232_2', 3: 'RS232_3', 4: 'RS485', 5: 'RS422', 6: 'TTL'}
    IMU_COM_TYPES_REV = {'None': 0, 'RS232_1': 1, 'RS232_2': 2, 'RS232_3': 3, 'RS485': 4, 'RS422': 5, 'TTL': 6}

    BISS_COM_TYPES = {0: 'None', 1: 'SPI1', 2: 'SPI2'}
    BISS_COM_TYPES_REV = {'None': 0, 'SPI1': 1, 'SPI2': 2}

    CAN_BAUD_RATES = {0: 'None', 1: '125K', 2: '250K', 3: '500K', 4: '1M'}
    CAN_BAUD_RATES_REV = {'None': 0, '125K': 1, '250K': 2, '500K': 3, '1M': 4}

    MULTI_COM_BITS = {0: 'Ethernet', 1: 'RS232', 2: 'RS422', 3: 'TTL'}

    def __init__(self):
        super().__init__()
        self.driver = None
        self.motors = {}
        self.is_connected = False
        self.num_axes = 2
        self.param_widgets = {}
        self.system_widgets = {}
        self.packet_counter = 0
        self.metadata_info = {}

        self.setWindowTitle("Captrack Configuration Tool")
        self.setMinimumSize(1450, 920)
        self._setup_ui()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(8)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Connection bar
        main_layout.addWidget(self._create_connection_bar())

        # Info bar (shows metadata from JSON)
        self.info_frame = self._create_info_bar()
        main_layout.addWidget(self.info_frame)

        # Splitter: config | packet log
        splitter = QSplitter(Qt.Horizontal)
        self.config_tabs = QTabWidget()
        self._build_all_tabs()
        splitter.addWidget(self.config_tabs)
        splitter.addWidget(self._create_packet_log())
        splitter.setSizes([850, 500])
        main_layout.addWidget(splitter)

        # Action bar
        main_layout.addWidget(self._create_action_bar())
        self.statusBar().showMessage("Ready — not connected")

    def _create_connection_bar(self):
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { background-color: #111827; border: 1px solid #1e293b; "
            "border-radius: 8px; }")
        frame.setFixedHeight(54)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(14, 6, 14, 6)
        layout.setSpacing(8)

        lbl = QLabel("IP:")
        lbl.setStyleSheet("color: #93c5fd; font-weight: bold; font-size: 11pt;")
        layout.addWidget(lbl)

        self.ip_fields = []
        for i in range(4):
            f = QSpinBox()
            f.setRange(0, 255)
            f.setValue([192, 168, 10, 86][i])
            f.setFixedWidth(60)
            self.ip_fields.append(f)
            layout.addWidget(f)
            if i < 3:
                d = QLabel(".")
                d.setFixedWidth(6)
                d.setStyleSheet("color: #93c5fd; font-size: 14pt; font-weight: bold;")
                layout.addWidget(d)

        layout.addSpacing(16)
        lbl = QLabel("Port:")
        lbl.setStyleSheet("color: #93c5fd; font-weight: bold; font-size: 11pt;")
        layout.addWidget(lbl)
        self.port_field = QSpinBox()
        self.port_field.setRange(1, 65535)
        self.port_field.setValue(4949)
        self.port_field.setFixedWidth(80)
        layout.addWidget(self.port_field)

        layout.addSpacing(20)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.setFixedWidth(120)
        self.btn_connect.clicked.connect(self._toggle_connection)
        layout.addWidget(self.btn_connect)

        self.conn_indicator = QLabel("  OFFLINE  ")
        self.conn_indicator.setStyleSheet(
            "background-color: #450a0a; color: #fca5a5; padding: 5px 14px; "
            "border-radius: 5px; font-weight: bold; font-size: 10pt; "
            "border: 1px solid #7f1d1d;")
        layout.addWidget(self.conn_indicator)

        layout.addStretch()
        return frame

    def _create_info_bar(self):
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { background-color: #111827; border: 1px solid #1e293b; "
            "border-radius: 8px; }")
        frame.setFixedHeight(36)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(14, 2, 14, 2)
        layout.setSpacing(20)

        self.info_labels = {}
        for key, default in [("customer", "—"), ("project", "—"),
                             ("axes", "—"), ("created", "—")]:
            lbl_name = QLabel(f"{key.capitalize()}:")
            lbl_name.setObjectName("info_label")
            lbl_val = QLabel(default)
            lbl_val.setObjectName("info_value")
            layout.addWidget(lbl_name)
            layout.addWidget(lbl_val)
            self.info_labels[key] = lbl_val
            layout.addSpacing(10)

        layout.addStretch()
        return frame

    def _update_info_bar(self):
        self.info_labels["customer"].setText(self.metadata_info.get("customer", "—") or "—")
        self.info_labels["project"].setText(self.metadata_info.get("project", "—") or "—")
        self.info_labels["axes"].setText(str(self.metadata_info.get("num_axes", self.num_axes)))
        created = self.metadata_info.get("created", "")
        self.info_labels["created"].setText(created[:16] if created else "—")

    def _create_packet_log(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)

        header = QLabel("  PACKET LOG")
        header.setStyleSheet(
            "font-weight: bold; font-size: 11pt; color: #93c5fd; "
            "padding: 8px 10px; background-color: #111827; "
            "border: 1px solid #1e293b; border-radius: 6px;")
        layout.addWidget(header)

        self.packet_tree = QTreeWidget()
        self.packet_tree.setHeaderLabels(["#", "Dir", "Function", "Packet Data (Hex)"])
        self.packet_tree.setColumnWidth(0, 42)
        self.packet_tree.setColumnWidth(1, 36)
        self.packet_tree.setColumnWidth(2, 190)
        self.packet_tree.header().setStretchLastSection(True)
        self.packet_tree.setAlternatingRowColors(True)
        self.packet_tree.setRootIsDecorated(False)
        layout.addWidget(self.packet_tree)

        btn_clear = QPushButton("Clear Log")
        btn_clear.clicked.connect(lambda: (self.packet_tree.clear(), setattr(self, 'packet_counter', 0)))
        layout.addWidget(btn_clear)
        return widget

    def _create_action_bar(self):
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { background-color: #111827; border: 1px solid #1e293b; "
            "border-radius: 8px; }")
        frame.setFixedHeight(54)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(14, 6, 14, 6)
        layout.setSpacing(12)

        self.btn_read_all = QPushButton("Read All from System")
        self.btn_read_all.setEnabled(False)
        self.btn_read_all.setMinimumWidth(180)
        self.btn_read_all.setStyleSheet("font-size: 10pt; padding: 6px 16px;")
        self.btn_read_all.clicked.connect(self._read_all_config)
        layout.addWidget(self.btn_read_all)

        self.btn_write_all = QPushButton("Write All to System")
        self.btn_write_all.setEnabled(False)
        self.btn_write_all.setMinimumWidth(180)
        self.btn_write_all.setStyleSheet("font-size: 10pt; padding: 6px 16px;")
        self.btn_write_all.clicked.connect(self._write_all_config)
        layout.addWidget(self.btn_write_all)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet("color: #334155;")
        layout.addWidget(sep)

        self.btn_save_json = QPushButton("Save JSON")
        self.btn_save_json.setMinimumWidth(120)
        self.btn_save_json.setStyleSheet("font-size: 10pt; padding: 6px 16px;")
        self.btn_save_json.clicked.connect(self._save_json)
        layout.addWidget(self.btn_save_json)

        self.btn_load_json = QPushButton("Load JSON")
        self.btn_load_json.setMinimumWidth(120)
        self.btn_load_json.setStyleSheet("font-size: 10pt; padding: 6px 16px;")
        self.btn_load_json.clicked.connect(self._load_json)
        layout.addWidget(self.btn_load_json)

        layout.addStretch()

        self.btn_cfg_save = QPushButton("cfg_save (Flash)")
        self.btn_cfg_save.setEnabled(False)
        self.btn_cfg_save.setMinimumWidth(150)
        self.btn_cfg_save.setStyleSheet(
            "font-size: 10pt; padding: 6px 16px; "
            "background-color: #78350f; border-color: #d97706; color: #fcd34d;")
        self.btn_cfg_save.clicked.connect(self._cfg_save_all)
        layout.addWidget(self.btn_cfg_save)

        return frame

    # ========== Tab building ==========

    def _build_all_tabs(self):
        self.config_tabs.clear()
        self.param_widgets = {}

        for axis in range(1, self.num_axes + 1):
            self.param_widgets[axis] = {}
            self.config_tabs.addTab(self._create_axis_tab(axis), f"  Axis {axis}  ")

        self.system_widgets = {}
        self.config_tabs.addTab(self._create_system_tab(), "  System  ")

    def _rebuild_tabs_for_axes(self, new_num):
        if new_num == self.num_axes:
            return
        self.num_axes = new_num
        self._build_all_tabs()

    def _create_axis_tab(self, axis_num):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(12)
        layout.setContentsMargins(10, 10, 10, 10)

        # --- Parameters from Technosoft ---
        grp = QGroupBox("Parameters from Technosoft Setup")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_combo_row(g, row, axis_num, "motor_type", "Motor Type",
                            list(self.MOTOR_TYPES.values()), "get_cfg_Motor_Type", "set_cfg_Motor_Type", "motor_type")
        row += 1
        self._add_float_row(g, row, axis_num, "gear_ratio", "Gear Ratio", 0, 100000,
                            "get_cfg_Gear_Ratio", "set_cfg_Gear_Ratio")
        row += 1
        self._add_float_row(g, row, axis_num, "max_vdc", "Max VDC (V)", 0, 200,
                            "get_cfg_Max_VDC", "set_cfg_Max_VDC")
        row += 1
        self._add_float_row(g, row, axis_num, "peak_current", "Peak Current (A)", 0, 100,
                            "get_cfg_Peak_Current", "set_cfg_Peak_Current")
        row += 1
        self._add_float_row(g, row, axis_num, "slow_loop_sampling", "Slow Loop Sampling", 0, 100000,
                            "get_cfg_slow_loop_sampling", "set_cfg_slow_loop_sampling")
        row += 1
        self._add_int_row(g, row, axis_num, "micro_steps", "Micro Steps", 0, 65535,
                          "get_cfg_micro_Steps", "set_cfg_micro_Steps")
        row += 1
        self._add_int_row(g, row, axis_num, "steps_per_rev", "Steps Per Rev", 0, 65535,
                          "get_cfg_steps_Per_Rev", "set_cfg_steps_Per_Rev")
        row += 1
        self._add_combo_row(g, row, axis_num, "close_loop_enable", "Close Loop",
                            list(self.CLOSE_LOOP.values()),
                            "get_cfg_close_loop_enable", "set_cfg_close_loop_enable", "close_loop")
        row += 1
        self._add_float_row(g, row, axis_num, "encoder_lines", "Encoder Lines", 0, 100000,
                            "get_cfg_Encoder_Lines", "set_cfg_Encoder_Lines")
        row += 1
        self._add_float_row(g, row, axis_num, "load_encoder_lines", "Load Enc Lines", 0, 100000,
                            "get_cfg_Load_Encoder_Lines", "set_cfg_Load_Encoder_Lines")
        row += 1
        self._add_combo_row(g, row, axis_num, "encoder_location", "Encoder Location",
                            list(self.ENCODER_LOCATIONS.values()),
                            "get_encoder_Location", "set_encoder_Location", "encoder_location")
        row += 1
        self._add_combo_row(g, row, axis_num, "apos_load_type", "Apos Load Type",
                            list(self.APOS_LOAD_TYPES.values()),
                            "get_cfg_Apos_Load_Type", "set_cfg_Apos_Load_Type", "apos_load_type")
        layout.addWidget(grp)

        # --- BISS Configuration ---
        grp = QGroupBox("BISS Encoder Configuration")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_int_row(g, row, axis_num, "biss_resolution", "BISS Resolution", 0, 255,
                          "get_cfg_Biss_Resolution", "set_cfg_Biss_Resolution")
        row += 1
        self._add_combo_row(g, row, axis_num, "biss_com", "BISS Port",
                            list(self.BISS_COM_TYPES.values()),
                            "get_cfg_Biss_Com", "set_cfg_Biss_Com", "biss_com")
        row += 1
        self._add_float_row(g, row, axis_num, "abs_enc_offset", "Abs Enc Offset (°)", -360, 360,
                            "get_cfg_Abs_Enc_Offset", "set_cfg_Abs_Enc_Offset")
        row += 1
        self._add_int_row(g, row, axis_num, "abs_enc_reverse", "Abs Enc Reverse", 0, 1,
                          "get_cfg_Abs_Enc_Reverse", "set_cfg_Abs_Enc_Reverse")
        layout.addWidget(grp)

        # --- Limits Motion Parameters ---
        grp = QGroupBox("Limits — Motion Parameters")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_float_row(g, row, axis_num, "max_speed", "Max Speed", 0, 100000,
                            "get_cfg_Max_Speed", "set_cfg_Max_Speed")
        row += 1
        self._add_float_row(g, row, axis_num, "min_speed", "Min Speed", 0, 100000,
                            "get_cfg_Min_Speed", "set_cfg_Min_Speed")
        row += 1
        self._add_float_row(g, row, axis_num, "max_acceleration", "Max Acceleration", 0, 65535,
                            "get_cfg_max_Acceleration", "set_cfg_max_Acceleration")
        row += 1
        self._add_float_row(g, row, axis_num, "min_acceleration", "Min Acceleration", 0, 100000,
                            "get_cfg_Min_Acceleration", "set_cfg_Min_Acceleration")
        layout.addWidget(grp)

        layout.addStretch()
        scroll.setWidget(container)
        return scroll

    def _create_system_tab(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setSpacing(12)
        layout.setContentsMargins(10, 10, 10, 10)

        # --- Communication Info ---
        grp = QGroupBox("Communication Info")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_sys_combo_row(g, row, "pc_com", "PC Communication",
                                list(self.PC_COM_TYPES.values()),
                                "get_cfp_PC_COM", "set_cfp_PC_COM", "pc_com")
        row += 1
        self._add_sys_combo_row(g, row, "ts_com_type", "TS Communication",
                                list(self.TS_COM_TYPES.values()),
                                "get_cfg_TS_COM_TYPE", "set_cfg_TS_COM_TYPE", "ts_com_type")
        row += 1
        self._add_sys_combo_row(g, row, "can_baud_rate", "CAN Baud Rate",
                                list(self.CAN_BAUD_RATES.values()),
                                "get_cfg_Can_Baud_rate", "set_cfg_Can_Baud_rate", "can_baud_rate")
        row += 1
        self._add_sys_int_row(g, row, "com_axes_ts", "Com Axes TS", 0, 255,
                              "get_cfg_Com_Axes_TS", "set_cfg_Com_Axes_TS")
        row += 1
        self._add_sys_multi_com_row(g, row)
        layout.addWidget(grp)

        # --- IMU Configuration ---
        grp = QGroupBox("IMU Configuration")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_sys_combo_row(g, row, "reversed_imu", "IMU Source",
                                list(self.IMU_REVERSED.values()),
                                "get_cfg_reversed_IMU", "set_cfg_reversed_IMU", "imu_reversed")
        row += 1
        self._add_sys_combo_row(g, row, "imu_com", "IMU Communication",
                                list(self.IMU_COM_TYPES.values()),
                                "get_cfg_Imu_Com", "set_cfg_Imu_Com", "imu_com")
        row += 1
        self._add_sys_int_row(g, row, "imu_baud_rate", "IMU Baud Rate", 0, 10000000,
                              "get_cfg_Imu_Baud_Rate", "set_cfg_Imu_Baud_Rate")
        row += 1
        self._add_sys_int_row(g, row, "imu_freq", "IMU Frequency (Hz)", 0, 65535,
                              "get_cfg_Imu_Freq", "set_cfg_Imu_Freq")
        layout.addWidget(grp)

        # --- General Configuration ---
        grp = QGroupBox("General Configuration")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_sys_combo_row(g, row, "system_type", "System Type",
                                list(self.SYSTEM_TYPES.values()),
                                "get_cfg_System_Type", "set_cfg_System_Type", "system_type")
        row += 1
        self._add_sys_readonly_row(g, row, "firmware_version", "Firmware Version",
                                   "get_cfg_firmware_Version")
        row += 1
        self._add_sys_int_row(g, row, "serial_number", "Serial Number", 0, 2147483647,
                              "get_cfg_Serial_Number", "set_cfg_Serial_Number")
        layout.addWidget(grp)

        # --- IP Communication Commands ---
        grp = QGroupBox("IP Communication Commands")
        g = QGridLayout(grp)
        g.setSpacing(4)
        g.setColumnMinimumWidth(0, 170)
        g.setColumnMinimumWidth(1, 140)
        g.setColumnMinimumWidth(4, 210)
        row = 0
        self._add_sys_ip_row(g, row, "controller_ip", "Controller IP",
                             "GetControllerIP", "SetControllerIP")
        row += 1
        self._add_sys_int_row(g, row, "controller_port", "Controller Port", 1, 65535,
                              "GetControllerPort", "SetControllerPort")
        row += 1
        self._add_sys_ip_row(g, row, "controller_subnet", "Subnet Mask",
                             "GetControllerSubnetMask", "Set_Controller_Subnet_Mask")
        layout.addWidget(grp)

        layout.addStretch()
        scroll.setWidget(container)
        return scroll

    # ========== Row builders (axis) ==========

    def _add_float_row(self, grid, row, axis, key, label, min_v, max_v, get_method, set_method):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        widget = QDoubleSpinBox()
        widget.setRange(min_v, max_v)
        widget.setDecimals(4)
        widget.setFixedWidth(140)

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, a=axis, k=key, m=get_method: self._single_get(a, k, m))
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(lambda _, a=axis, k=key, m=set_method: self._single_set(a, k, m, "float"))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(widget, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.param_widgets[axis][key] = {
            "widget": widget, "status": status, "type": "float",
            "get_method": get_method, "set_method": set_method, "convert": "float"
        }

    def _add_int_row(self, grid, row, axis, key, label, min_v, max_v, get_method, set_method):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        widget = QSpinBox()
        widget.setRange(min_v, max_v)
        widget.setFixedWidth(140)

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, a=axis, k=key, m=get_method: self._single_get(a, k, m))
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(lambda _, a=axis, k=key, m=set_method: self._single_set(a, k, m, "int"))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(widget, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.param_widgets[axis][key] = {
            "widget": widget, "status": status, "type": "int",
            "get_method": get_method, "set_method": set_method, "convert": "int"
        }

    def _add_combo_row(self, grid, row, axis, key, label, options, get_method, set_method, convert):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        widget = QComboBox()
        widget.addItems(options)
        widget.setFixedWidth(140)
        widget.setMaxVisibleItems(15)

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, a=axis, k=key, m=get_method: self._single_get(a, k, m))
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(lambda _, a=axis, k=key, m=set_method: self._single_set(a, k, m, convert))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(widget, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.param_widgets[axis][key] = {
            "widget": widget, "status": status, "type": "combo",
            "get_method": get_method, "set_method": set_method, "convert": convert
        }

    # ========== Row builders (system) ==========

    def _add_sys_int_row(self, grid, row, key, label, min_v, max_v, get_method, set_method):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        widget = QSpinBox()
        widget.setRange(min_v, max_v)
        widget.setFixedWidth(140)

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, k=key, m=get_method: self._sys_single_get(k, m))
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(lambda _, k=key, m=set_method: self._sys_single_set(k, m, "int"))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(widget, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.system_widgets[key] = {
            "widget": widget, "status": status, "type": "int",
            "get_method": get_method, "set_method": set_method, "convert": "int"
        }

    def _add_sys_combo_row(self, grid, row, key, label, options, get_method, set_method, convert):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        widget = QComboBox()
        widget.addItems(options)
        widget.setFixedWidth(140)
        widget.setMaxVisibleItems(15)

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, k=key, m=get_method: self._sys_single_get(k, m))
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(lambda _, k=key, m=set_method: self._sys_single_set(k, m, convert))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(widget, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.system_widgets[key] = {
            "widget": widget, "status": status, "type": "combo",
            "get_method": get_method, "set_method": set_method, "convert": convert
        }

    def _add_sys_readonly_row(self, grid, row, key, label, get_method):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        widget = QLineEdit()
        widget.setReadOnly(True)
        widget.setFixedWidth(220)

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, k=key, m=get_method: self._sys_single_get(k, m))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(widget, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.system_widgets[key] = {
            "widget": widget, "status": status, "type": "readonly",
            "get_method": get_method, "set_method": None, "convert": None
        }

    def _add_sys_ip_row(self, grid, row, key, label, get_method, set_method):
        lbl = QLabel(label)
        lbl.setObjectName("param_label")
        container = QWidget()
        h = QHBoxLayout(container)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(2)
        ip_fields = []
        for i in range(4):
            f = QSpinBox()
            f.setRange(0, 255)
            f.setFixedWidth(54)
            ip_fields.append(f)
            h.addWidget(f)
            if i < 3:
                d = QLabel(".")
                d.setFixedWidth(6)
                h.addWidget(d)
        h.addStretch()

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(lambda _, k=key, m=get_method: self._sys_single_get(k, m))
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(lambda _, k=key, m=set_method: self._sys_single_set(k, m, "ip"))

        desc = QLabel(PARAM_DESCRIPTIONS.get(key, ""))
        desc.setObjectName("desc_label")
        status = QLabel("")
        status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(container, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(status, row, 5)

        self.system_widgets[key] = {
            "widget": ip_fields, "status": status, "type": "ip",
            "get_method": get_method, "set_method": set_method, "convert": "ip"
        }

    def _add_sys_multi_com_row(self, grid, row):
        """Multi Communication with checkboxes for each channel (bitmask)."""
        lbl = QLabel("Multi Communication")
        lbl.setObjectName("param_label")

        container = QWidget()
        h = QHBoxLayout(container)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(12)
        self.multi_com_checks = {}
        for bit, name in self.MULTI_COM_BITS.items():
            cb = QCheckBox(name)
            self.multi_com_checks[bit] = cb
            h.addWidget(cb)
        h.addStretch()

        btn_get = QPushButton("Get")
        btn_get.setObjectName("btn_get")
        btn_get.clicked.connect(self._multi_com_get)
        btn_set = QPushButton("Set")
        btn_set.setObjectName("btn_set")
        btn_set.clicked.connect(self._multi_com_set)

        desc = QLabel(PARAM_DESCRIPTIONS.get("multi_com", ""))
        desc.setObjectName("desc_label")

        self.multi_com_status = QLabel("")
        self.multi_com_status.setFixedWidth(75)

        grid.addWidget(lbl, row, 0)
        grid.addWidget(container, row, 1)
        grid.addWidget(btn_get, row, 2)
        grid.addWidget(btn_set, row, 3)
        grid.addWidget(desc, row, 4)
        grid.addWidget(self.multi_com_status, row, 5)

    def _multi_com_get(self):
        if not self.is_connected:
            self._show_not_connected()
            return
        motor = self.motors.get(1)
        if not motor:
            return
        result = self._call_motor_method(motor, "get_cfg_Multi_Com")
        if result is not None and isinstance(result, list):
            for bit, cb in self.multi_com_checks.items():
                cb.setChecked(self.MULTI_COM_BITS[bit] in result)
            self.multi_com_status.setText("OK")
            self.multi_com_status.setObjectName("status_ok")
            self.multi_com_status.style().unpolish(self.multi_com_status)
            self.multi_com_status.style().polish(self.multi_com_status)
        else:
            self.multi_com_status.setText(UNKNOWN)
            self.multi_com_status.setObjectName("status_err")
            self.multi_com_status.style().unpolish(self.multi_com_status)
            self.multi_com_status.style().polish(self.multi_com_status)

    def _multi_com_set(self):
        if not self.is_connected:
            self._show_not_connected()
            return
        motor = self.motors.get(1)
        if not motor:
            return
        bitmask = 0
        for bit, cb in self.multi_com_checks.items():
            if cb.isChecked():
                bitmask |= (1 << bit)
        result = self._call_motor_method(motor, "set_cfg_Multi_Com", bitmask)
        if result:
            self.multi_com_status.setText("SET OK")
            self.multi_com_status.setObjectName("status_ok")
        else:
            self.multi_com_status.setText("FAILED")
            self.multi_com_status.setObjectName("status_err")
        self.multi_com_status.style().unpolish(self.multi_com_status)
        self.multi_com_status.style().polish(self.multi_com_status)

    # ========== Packet logging ==========

    def _log_packet(self, direction, func_name, packet_data):
        self.packet_counter += 1
        item = QTreeWidgetItem()
        item.setText(0, str(self.packet_counter))
        item.setText(1, direction)
        item.setText(2, func_name)

        if isinstance(packet_data, (bytes, bytearray, list)):
            hex_str = " ".join(f"{b:02X}" for b in packet_data)
        elif packet_data is None:
            hex_str = "— NO RESPONSE —"
        else:
            hex_str = str(packet_data)

        item.setText(3, hex_str)

        colors = {"TX": "#60a5fa", "RX": "#34d399", "ERR": "#f87171"}
        c = QColor(colors.get(direction, "#d1d5db"))
        for col in range(4):
            item.setForeground(col, c)

        self.packet_tree.addTopLevelItem(item)
        self.packet_tree.scrollToBottom()

    def _call_motor_method(self, motor, method_name, *args):
        """Call motor method with TX/RX interception and retry."""
        original_send = motor.driver.send_and_receive

        def intercepted_send(command, buffer_size=1024, sleep_time=0.03):
            tx_data = command if isinstance(command, list) else list(command)
            self._log_packet("TX", method_name, tx_data)
            response = original_send(command, buffer_size, sleep_time)
            self._log_packet("RX", method_name, response)
            return response

        motor.driver.send_and_receive = intercepted_send
        result = None
        try:
            func = getattr(motor, method_name)
            for attempt in range(MAX_RETRIES):
                result = func(*args) if args else func()
                if result is not None:
                    break
                time.sleep(RETRY_DELAY_MS / 1000.0)
        except Exception as e:
            self._log_packet("ERR", method_name, str(e).encode())
            result = None
        finally:
            motor.driver.send_and_receive = original_send

        return result

    # ========== Single Get/Set ==========

    def _set_status(self, status_label, text, state):
        status_label.setText(text)
        status_label.setObjectName(f"status_{state}")
        status_label.style().unpolish(status_label)
        status_label.style().polish(status_label)

    def _single_get(self, axis, key, get_method):
        if not self.is_connected:
            self._show_not_connected()
            return
        motor = self.motors.get(axis)
        if not motor:
            return
        entry = self.param_widgets[axis][key]
        result = self._call_motor_method(motor, get_method)
        if result is not None:
            self._set_widget_value_entry(entry, result)
            self._set_status(entry["status"], "OK", "ok")
        else:
            self._set_status(entry["status"], UNKNOWN, "err")

    def _single_set(self, axis, key, set_method, convert):
        if not self.is_connected:
            self._show_not_connected()
            return
        motor = self.motors.get(axis)
        if not motor:
            return
        entry = self.param_widgets[axis][key]
        raw_val = self._get_widget_value_entry(entry)
        value = self._convert_for_write(raw_val, convert)
        result = self._call_motor_method(motor, set_method, value)
        if result:
            self._set_status(entry["status"], "SET OK", "ok")
        else:
            self._set_status(entry["status"], "FAILED", "err")

    def _sys_single_get(self, key, get_method):
        if not self.is_connected:
            self._show_not_connected()
            return
        motor = self.motors.get(1)
        if not motor:
            return
        entry = self.system_widgets[key]
        result = self._call_motor_method(motor, get_method)
        if result is not None:
            self._set_widget_value_entry(entry, result)
            self._set_status(entry["status"], "OK", "ok")
        else:
            self._set_status(entry["status"], UNKNOWN, "err")

    def _sys_single_set(self, key, set_method, convert):
        if not self.is_connected:
            self._show_not_connected()
            return
        motor = self.motors.get(1)
        if not motor:
            return
        entry = self.system_widgets[key]
        if entry.get("set_method") is None:
            return
        raw_val = self._get_widget_value_entry(entry)
        value = self._convert_for_write(raw_val, convert)
        result = self._call_motor_method(motor, set_method, value)
        if result:
            self._set_status(entry["status"], "SET OK", "ok")
        else:
            self._set_status(entry["status"], "FAILED", "err")

    # ========== Widget helpers ==========

    def _set_widget_value_entry(self, entry, value):
        wtype = entry["type"]
        widget = entry["widget"]
        try:
            if wtype == "float":
                widget.setValue(float(value))
            elif wtype == "int":
                widget.setValue(int(value))
            elif wtype == "combo":
                idx = widget.findText(str(value))
                if idx >= 0:
                    widget.setCurrentIndex(idx)
            elif wtype == "readonly":
                widget.setText(str(value))
            elif wtype == "ip":
                if isinstance(value, (list, tuple, bytearray, bytes)) and len(value) >= 4:
                    for i, f in enumerate(widget):
                        f.setValue(int(value[i]))
        except Exception:
            pass

    def _get_widget_value_entry(self, entry):
        wtype = entry["type"]
        widget = entry["widget"]
        if wtype == "float":
            return widget.value()
        elif wtype == "int":
            return widget.value()
        elif wtype == "combo":
            return widget.currentText()
        elif wtype == "readonly":
            return widget.text()
        elif wtype == "ip":
            return [f.value() for f in widget]
        return None

    def _convert_for_write(self, raw_value, convert):
        converters = {
            "float": lambda v: float(v),
            "int": lambda v: int(v),
            "motor_type": lambda v: self.MOTOR_TYPES_REV.get(v, 0),
            "system_type": lambda v: self.SYSTEM_TYPES_REV.get(v, 0),
            "encoder_location": lambda v: self.ENCODER_LOCATIONS_REV.get(v, 0),
            "apos_load_type": lambda v: self.APOS_LOAD_TYPES_REV.get(v, 0),
            "close_loop": lambda v: self.CLOSE_LOOP_REV.get(v, 0),
            "imu_reversed": lambda v: self.IMU_REVERSED_REV.get(v, 0),
            "imu_com": lambda v: self.IMU_COM_TYPES_REV.get(v, 0),
            "biss_com": lambda v: self.BISS_COM_TYPES_REV.get(v, 0),
            "can_baud_rate": lambda v: self.CAN_BAUD_RATES_REV.get(v, 0),
            "pc_com": lambda v: self.PC_COM_TYPES_REV.get(v, 0),
            "ts_com_type": lambda v: self.TS_COM_TYPES_REV.get(v, 0),
            "ip": lambda v: v,
        }
        fn = converters.get(convert)
        return fn(raw_value) if fn else raw_value

    # ========== Connection ==========

    def _toggle_connection(self):
        if self.is_connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        ip = ".".join(str(f.value()) for f in self.ip_fields)
        port = self.port_field.value()

        self._log_packet("TX", "TCP_CONNECT", f"{ip}:{port}".encode())
        try:
            self.driver = Controller(host=ip, port=port, timeout=5.0)
            result = self.driver.connect()
            if result:
                self.is_connected = True
                self.motors = {}
                for ax in range(1, self.num_axes + 1):
                    self.motors[ax] = MotorControl(
                        driver=self.driver, axis_number=ax,
                        max_speed=20, position_units="degrees"
                    )
                self._log_packet("RX", "TCP_CONNECT", b"ACK - Connected")
                self.conn_indicator.setText("  ONLINE  ")
                self.conn_indicator.setStyleSheet(
                    "background-color: #052e16; color: #6ee7b7; padding: 5px 14px; "
                    "border-radius: 5px; font-weight: bold; font-size: 10pt; "
                    "border: 1px solid #065f46;")
                self.btn_connect.setText("Disconnect")
                self.btn_connect.setObjectName("btn_disconnect")
                self.btn_connect.style().unpolish(self.btn_connect)
                self.btn_connect.style().polish(self.btn_connect)
                self.btn_read_all.setEnabled(True)
                self.btn_write_all.setEnabled(True)
                self.btn_cfg_save.setEnabled(True)
                self.statusBar().showMessage(f"Connected to {ip}:{port} — {self.num_axes} axes")
            else:
                self._log_packet("ERR", "TCP_CONNECT", b"Connection refused or timeout")
        except Exception as e:
            self._log_packet("ERR", "TCP_CONNECT", str(e).encode())

    def _disconnect(self):
        if self.driver:
            self.driver.disconnect()
        self.driver = None
        self.motors = {}
        self.is_connected = False
        self.conn_indicator.setText("  OFFLINE  ")
        self.conn_indicator.setStyleSheet(
            "background-color: #450a0a; color: #fca5a5; padding: 5px 14px; "
            "border-radius: 5px; font-weight: bold; font-size: 10pt; "
            "border: 1px solid #7f1d1d;")
        self.btn_connect.setText("Connect")
        self.btn_connect.setObjectName("btn_connect")
        self.btn_connect.style().unpolish(self.btn_connect)
        self.btn_connect.style().polish(self.btn_connect)
        self.btn_read_all.setEnabled(False)
        self.btn_write_all.setEnabled(False)
        self.btn_cfg_save.setEnabled(False)
        self.statusBar().showMessage("Disconnected")
        self._log_packet("TX", "TCP_DISCONNECT", b"Closed")

    def _show_not_connected(self):
        QMessageBox.warning(self, "Not Connected",
                            "Please connect to the controller first.")

    # ========== Bulk operations ==========

    def _read_all_config(self):
        if not self.is_connected:
            self._show_not_connected()
            return

        self.statusBar().showMessage("Loading configuration from controller...")
        QApplication.processEvents()

        for ax in range(1, self.num_axes + 1):
            self._call_motor_method(self.motors[ax], "cfg_load")

        # 1 second delay after cfg_load to let the controller settle
        time.sleep(CFG_LOAD_DELAY)
        QApplication.processEvents()

        for ax in range(1, self.num_axes + 1):
            motor = self.motors[ax]
            for key, entry in self.param_widgets.get(ax, {}).items():
                get_method = entry.get("get_method")
                if get_method:
                    result = self._call_motor_method(motor, get_method)
                    if result is not None:
                        self._set_widget_value_entry(entry, result)
                        self._set_status(entry["status"], "OK", "ok")
                    else:
                        self._set_status(entry["status"], UNKNOWN, "err")
                    QApplication.processEvents()

        motor = self.motors[1]
        for key, entry in self.system_widgets.items():
            get_method = entry.get("get_method")
            if get_method:
                result = self._call_motor_method(motor, get_method)
                if result is not None:
                    self._set_widget_value_entry(entry, result)
                    self._set_status(entry["status"], "OK", "ok")
                else:
                    self._set_status(entry["status"], UNKNOWN, "err")
                QApplication.processEvents()

        # Also read multi com
        self._multi_com_get()

        self.statusBar().showMessage("Read all configuration complete")

    def _write_all_config(self):
        if not self.is_connected:
            self._show_not_connected()
            return

        reply = QMessageBox.question(
            self, "Confirm Write",
            "Write ALL parameters to the controller?\n\n"
            "This will overwrite the current configuration.",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        for ax in range(1, self.num_axes + 1):
            motor = self.motors[ax]
            for key, entry in self.param_widgets.get(ax, {}).items():
                set_method = entry.get("set_method")
                convert = entry.get("convert")
                if set_method:
                    raw_val = self._get_widget_value_entry(entry)
                    value = self._convert_for_write(raw_val, convert)
                    result = self._call_motor_method(motor, set_method, value)
                    if result:
                        self._set_status(entry["status"], "SET OK", "ok")
                    else:
                        self._set_status(entry["status"], "FAILED", "err")
                    QApplication.processEvents()

        motor = self.motors[1]
        for key, entry in self.system_widgets.items():
            set_method = entry.get("set_method")
            convert = entry.get("convert")
            if set_method:
                raw_val = self._get_widget_value_entry(entry)
                value = self._convert_for_write(raw_val, convert)
                result = self._call_motor_method(motor, set_method, value)
                if result:
                    self._set_status(entry["status"], "SET OK", "ok")
                else:
                    self._set_status(entry["status"], "FAILED", "err")
                QApplication.processEvents()

        self.statusBar().showMessage("Write all configuration complete")

    def _cfg_save_all(self):
        if not self.is_connected:
            self._show_not_connected()
            return
        for ax in range(1, self.num_axes + 1):
            self._call_motor_method(self.motors[ax], "cfg_save")
        self.statusBar().showMessage("cfg_save complete — configuration saved to flash")

    # ========== JSON ==========

    def _save_json(self):
        filepath, _ = QFileDialog.getSaveFileName(
            self, "Save Configuration", "", "JSON Files (*.json)")
        if not filepath:
            return

        config = {
            "metadata": {
                "created": datetime.now().isoformat(),
                "application": "Captrack Configuration Tool",
                "version": "2.0",
                "num_axes": self.num_axes,
                "customer": self.metadata_info.get("customer", ""),
                "project": self.metadata_info.get("project", ""),
            },
            "system": {},
        }

        for key, entry in self.system_widgets.items():
            config["system"][key] = self._get_widget_value_entry(entry)

        # Save multi_com state
        multi_com_active = []
        for bit, cb in self.multi_com_checks.items():
            if cb.isChecked():
                multi_com_active.append(self.MULTI_COM_BITS[bit])
        config["system"]["multi_com"] = multi_com_active

        for ax in range(1, self.num_axes + 1):
            axis_key = f"axis_{ax}"
            config[axis_key] = {}
            for key, entry in self.param_widgets.get(ax, {}).items():
                config[axis_key][key] = self._get_widget_value_entry(entry)

        try:
            with open(filepath, 'w') as f:
                json.dump(config, f, indent=4)
            self.statusBar().showMessage(f"Saved: {filepath}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save:\n{e}")

    def _load_json(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "Load Configuration", "", "JSON Files (*.json)")
        if not filepath:
            return

        try:
            with open(filepath, 'r') as f:
                config = json.load(f)
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to read JSON:\n{e}")
            return

        # Extract and display metadata
        meta = config.get("metadata", {})
        self.metadata_info = {
            "customer": meta.get("customer", ""),
            "project": meta.get("project", ""),
            "num_axes": meta.get("num_axes", self.num_axes),
            "created": meta.get("created", ""),
        }
        self._update_info_bar()

        # Adjust axes from JSON
        json_axes = meta.get("num_axes")
        if json_axes and json_axes != self.num_axes:
            self._rebuild_tabs_for_axes(json_axes)
            if self.is_connected:
                self.motors = {}
                for ax in range(1, self.num_axes + 1):
                    self.motors[ax] = MotorControl(
                        driver=self.driver, axis_number=ax,
                        max_speed=20, position_units="degrees"
                    )

        errors = []

        # Load system params
        sys_cfg = config.get("system", {})
        for key, value in sys_cfg.items():
            if key == "multi_com":
                if isinstance(value, list):
                    for bit, cb in self.multi_com_checks.items():
                        cb.setChecked(self.MULTI_COM_BITS[bit] in value)
                continue
            if key in self.system_widgets:
                try:
                    self._set_widget_value_entry(self.system_widgets[key], value)
                    self._set_status(self.system_widgets[key]["status"], "Loaded", "loaded")
                except Exception as e:
                    errors.append(f"system/{key}: {e}")
            else:
                errors.append(f"system/{key}: not recognized")

        # Load axis params
        for ax in range(1, self.num_axes + 1):
            axis_key = f"axis_{ax}"
            ax_cfg = config.get(axis_key, {})
            for key, value in ax_cfg.items():
                if ax in self.param_widgets and key in self.param_widgets[ax]:
                    try:
                        self._set_widget_value_entry(self.param_widgets[ax][key], value)
                        self._set_status(self.param_widgets[ax][key]["status"], "Loaded", "loaded")
                    except Exception as e:
                        errors.append(f"axis_{ax}/{key}: {e}")
                else:
                    errors.append(f"axis_{ax}/{key}: not recognized")

        if errors:
            QMessageBox.warning(self, "Load Warnings",
                                "Some parameters could not be loaded:\n\n" +
                                "\n".join(errors[:20]))

        self.statusBar().showMessage(f"Loaded: {os.path.basename(filepath)}")

    def closeEvent(self, event):
        if self.is_connected:
            self._disconnect()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLESHEET)
    window = ConfigApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
