#!/usr/bin/env python3
"""SimpleFOC CAN Studio - GUI для сервопривода FOCG431.

Весь обмен идёт напрямую CAN-кадрами по протоколу CANCommander (класс
RawRegisterClient). pysimplefoc используется только как источник значений
перечислений режимов - никакого ввода-вывода через него не происходит.
"""
import os
import re
import sys
import struct
import subprocess
import threading
import time
from collections import deque, OrderedDict

import tkinter as tk
from tkinter import filedialog
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import can as pycan

from simplefoc import MotionControlType, TorqueControlType

# ============================ ПРОТОКОЛ ============================
# Формат extended (29-бит) CAN ID (см. CANCommander.h прошивки):
#   [27:20] адрес узла, [19:16] тип пакета, [15:8] номер регистра, [7:0] индекс мотора
CAN_ADDRESS_SHIFT = 20
CAN_PACKET_TYPE_SHIFT = 16
CAN_REGISTER_SHIFT = 8
PKT_READ_REQUEST = 0x1
PKT_WRITE_REQUEST = 0x2
PKT_READ_RESPONSE = 0x3

CAN_BITRATE = 1000000

# --- Штатные регистры SimpleFOC (simplefoc/registers.py) ---
REG_STATUS          = 0x00
REG_TARGET          = 0x01
REG_ENABLE          = 0x04
REG_CONTROL_MODE    = 0x05
REG_TORQUE_MODE     = 0x06
REG_ANGLE           = 0x09
REG_VELOCITY        = 0x11
REG_CURRENT_Q       = 0x22
REG_CURRENT_D       = 0x23
REG_VEL_PID_P       = 0x30
REG_VEL_PID_I       = 0x31
REG_VEL_PID_D       = 0x32
REG_VEL_PID_RAMP    = 0x34
REG_VEL_LPF_T       = 0x35
REG_ANG_PID_P       = 0x36
REG_ANG_PID_I       = 0x37
REG_ANG_PID_D       = 0x38
REG_VOLTAGE_LIMIT   = 0x50
REG_CURRENT_LIMIT   = 0x51
REG_VELOCITY_LIMIT  = 0x52
REG_POLE_PAIRS      = 0x63
REG_PHASE_RESISTANCE = 0x64
REG_KV              = 0x65

# --- Кастомные регистры прошивки (servo_can_bridge.cpp) ---
REG_GEAR_RATIO   = 0xE0
REG_SERVO_TARGET = 0xE1
REG_SERVO_ANGLE  = 0xE2
REG_SERVO_VEL    = 0xE3
REG_VOLTAGE      = 0xE4
REG_TEMPERATURE  = 0xE5
REG_CANID        = 0xE6
REG_REBOOT       = 0xE7
REG_ENC_ANGLE    = 0xE8
REG_ENC_RPM      = 0xE9
REG_ENC_TEMP     = 0xEA
REG_ENC_STATUS   = 0xEB
REG_BOOTLOADER   = 0xEC

# Значение, которым SimpleFOC отдаёт "параметр не задан"
NOT_SET = -12345.0

# Биты статуса энкодера MBS (регистр 0xEB)
ENC_STATUS_BITS = [
    (0, "over speed"),
    (1, "температура вне диапазона"),
    (2, "слабое магнитное поле"),
    (3, "сильное магнитное поле"),
    (4, "низкий заряд батареи"),
    (5, "потеря батареи (multiturn сброшен)"),
    (6, "WARNING"),
    (7, "ERROR (данные невалидны)"),
]

MOTION_MODES = ["torque", "velocity", "angle", "velocity_openloop", "angle_openloop"]
TORQUE_MODES = ["voltage", "dc_current", "foc_current"]

DEFAULT_FLASHTOOL = os.path.expanduser("~/katapult/scripts/flashtool.py")
DEFAULT_FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware.bin")

# ==================== СИГНАЛЫ ДЛЯ ОПРОСА И ГРАФИКА ====================
# key -> (подпись, регистр, формат, единицы, цвет линии на графике)
SIGNALS = OrderedDict([
    ("angle",       ("Угол мотора",           REG_ANGLE,       'f', "rad",   "#2ecc71")),
    ("velocity",    ("Скорость мотора",       REG_VELOCITY,    'f', "rad/s", "#27ae60")),
    ("servo_angle", ("Угол после редуктора",  REG_SERVO_ANGLE, 'f', "rad",   "#f1c40f")),
    ("servo_vel",   ("Скорость редуктора",    REG_SERVO_VEL,   'f', "rad/s", "#f39c12")),
    ("voltage",     ("Напряжение питания",    REG_VOLTAGE,     'f', "V",     "#3498db")),
    ("temp",        ("Температура мотора",    REG_TEMPERATURE, 'f', "°C",    "#e67e22")),
    ("curr_q",      ("Ток Q",                 REG_CURRENT_Q,   'f', "A",     "#9b59b6")),
    ("curr_d",      ("Ток D",                 REG_CURRENT_D,   'f', "A",     "#8e44ad")),
    ("enc_angle",   ("Угол энкодера",         REG_ENC_ANGLE,   'f', "rad",   "#1abc9c")),
    ("enc_rpm",     ("Скорость энкодера",     REG_ENC_RPM,     'f', "об/с",  "#16a085")),
    ("enc_temp",    ("Температура энкодера",  REG_ENC_TEMP,    'f', "°C",    "#d35400")),
])

# Сколько точек держим в буфере графика и ширина окна отображения по умолчанию
PLOT_POINTS = 3000
DEFAULT_PLOT_WINDOW_S = 10

# Минимальная высота оси Y по единицам измерения. Без неё автомасштаб
# растягивает шум (напряжение ±0.1 В) на весь график и он нечитаем.
MIN_Y_SPAN = {
    "V": 2.0, "°C": 10.0, "rad": 0.5, "rad/s": 2.0, "A": 1.0, "об/с": 2.0,
}
# Сглаживание отображения (экспоненциальное): 1.0 - без фильтра
SMOOTH_ALPHA = 0.25

# Фоновый «лайт-опрос»: телеметрия 2 раза в секунду. Держит показания живыми
# при минимальной нагрузке; настраиваемый авто-опрос работает независимо.
LIGHT_POLL_MS = 500
TELEM_KEYS = ("voltage", "temp", "enc_angle", "enc_rpm", "enc_temp")


class RawRegisterClient:
    """Чтение/запись регистров сырыми CAN-кадрами по протоколу CANCommander."""

    def __init__(self, channel, node_addr, motor_idx=0):
        self.node = node_addr
        self.motor = motor_idx
        self.bus = pycan.Bus(channel=channel, interface="socketcan")
        # Аппаратно принимаем только ответы (тип 0x3) нашего узла
        resp_id = (node_addr << CAN_ADDRESS_SHIFT) | (PKT_READ_RESPONSE << CAN_PACKET_TYPE_SHIFT)
        resp_mask = (0xFF << CAN_ADDRESS_SHIFT) | (0xF << CAN_PACKET_TYPE_SHIFT)
        self.bus.set_filters([{"can_id": resp_id, "can_mask": resp_mask, "extended": True}])
        self.lock = threading.Lock()

    def close(self):
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def _make_id(self, ptype, reg):
        return ((self.node << CAN_ADDRESS_SHIFT) | (ptype << CAN_PACKET_TYPE_SHIFT)
                | (reg << CAN_REGISTER_SHIFT) | self.motor)

    @staticmethod
    def _pack(value, fmt):
        if fmt == 'f':
            return struct.pack('<f', float(value))
        if fmt == 'i':
            return struct.pack('<I', int(value) & 0xFFFFFFFF)
        return bytes([int(value) & 0xFF])

    @staticmethod
    def _unpack(data, fmt):
        if fmt == 'f':
            return struct.unpack('<f', data[:4])[0] if len(data) >= 4 else None
        if fmt == 'i':
            return struct.unpack('<I', data[:4])[0] if len(data) >= 4 else None
        return data[0] if data else None

    def write(self, reg, value, fmt='f'):
        """Запись значения в регистр (протокол не предусматривает подтверждения)."""
        msg = pycan.Message(arbitration_id=self._make_id(PKT_WRITE_REQUEST, reg),
                            is_extended_id=True, data=self._pack(value, fmt))
        with self.lock:
            self.bus.send(msg)

    def read(self, reg, fmt='f', timeout=0.3):
        """Чтение регистра. Возвращает значение либо None при таймауте."""
        with self.lock:
            while self.bus.recv(0) is not None:  # выгребаем старые кадры
                pass
            req = pycan.Message(arbitration_id=self._make_id(PKT_READ_REQUEST, reg),
                                is_extended_id=True, data=[])
            self.bus.send(req)
            deadline = time.time() + timeout
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                msg = self.bus.recv(remaining)
                if msg is None:
                    return None
                if not msg.is_extended_id:
                    continue
                aid = msg.arbitration_id
                if ((aid >> CAN_PACKET_TYPE_SHIFT) & 0xF) != PKT_READ_RESPONSE: continue
                if ((aid >> CAN_ADDRESS_SHIFT) & 0xFF) != self.node: continue
                if ((aid >> CAN_REGISTER_SHIFT) & 0xFF) != reg: continue
                return self._unpack(bytes(msg.data), fmt)


def scan_node_ids(channel, listen=0.6, gap=0.002):
    """Поиск узлов на шине.

    Рассылает запрос чтения REG_STATUS по всем адресам 1..254 и собирает ответы.
    Аппаратный фильтр каждого узла пропускает только его собственный адрес,
    поэтому рассылка не создаёт лавины на приёмной стороне. Адрес узла берётся
    из ID ответного кадра.
    """
    bus = pycan.Bus(channel=channel, interface="socketcan")
    try:
        # принимаем ответы любого узла
        bus.set_filters([{"can_id": PKT_READ_RESPONSE << CAN_PACKET_TYPE_SHIFT,
                          "can_mask": 0xF << CAN_PACKET_TYPE_SHIFT, "extended": True}])
        found = []
        for addr in range(1, 255):
            aid = ((addr << CAN_ADDRESS_SHIFT) | (PKT_READ_REQUEST << CAN_PACKET_TYPE_SHIFT)
                   | (REG_STATUS << CAN_REGISTER_SHIFT))
            try:
                bus.send(pycan.Message(arbitration_id=aid, is_extended_id=True, data=[]))
            except pycan.CanError:
                pass
            # ответы забираем на лету, чтобы не переполнить приёмный буфер
            while True:
                msg = bus.recv(gap)
                if msg is None:
                    break
                a = (msg.arbitration_id >> CAN_ADDRESS_SHIFT) & 0xFF
                if a not in found:
                    found.append(a)
        deadline = time.time() + listen
        while time.time() < deadline:
            msg = bus.recv(deadline - time.time())
            if msg is None:
                break
            a = (msg.arbitration_id >> CAN_ADDRESS_SHIFT) & 0xFF
            if a not in found:
                found.append(a)
        return sorted(found)
    finally:
        try:
            bus.shutdown()
        except Exception:
            pass


ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")


class SimpleFOCCANStudio(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("SimpleFOC CAN Studio PRO")
        self.geometry("1280x900")

        self.raw = None
        self.running = False
        self.motor_enabled = False
        self.auto_poll_enabled = False
        self.poll_interval = 100
        self.poll_thread = None
        self.light_poll_active = False
        self.light_thread = None

        # Буферы графика: key -> deque[(t, value)]
        self.data = {k: deque(maxlen=PLOT_POINTS) for k in SIGNALS}
        self.plot_lines = {}
        self.axis_of = {}
        self.plot_window_s = DEFAULT_PLOT_WINDOW_S
        self.autoscroll_var = tk.BooleanVar(value=True)
        self.smooth_var = tk.BooleanVar(value=True)
        self.axis_span = {}
        self.start_time = time.time()

        # Переменные чекбоксов "опрашивать" / "на график"
        self.poll_vars = {k: tk.BooleanVar(value=(k == "angle")) for k in SIGNALS}
        self.plot_vars = {k: tk.BooleanVar(value=(k in ("angle", "voltage"))) for k in SIGNALS}
        self.value_labels = {}

        self.create_layout()
        self.scan_can_interfaces()

    # ======================= РАЗМЕТКА =======================

    def create_layout(self):
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=4)
        self.grid_columnconfigure(1, weight=5)

        left_panel = ctk.CTkFrame(self, corner_radius=10)
        left_panel.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # --- подключение ---
        conn_frame = ctk.CTkFrame(left_panel)
        conn_frame.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(conn_frame, text="ПОДКЛЮЧЕНИЕ ШИНЫ",
                     font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=4,
                                                         sticky="w", padx=10, pady=5)

        self.can_cb = ctk.CTkComboBox(conn_frame, values=["can0"], width=100)
        self.can_cb.grid(row=1, column=0, padx=5, pady=5)

        self.addr_cb = ctk.CTkComboBox(conn_frame, values=["1"], width=80)
        self.addr_cb.set("201")
        self.addr_cb.grid(row=1, column=1, padx=5, pady=5)

        self.btn_scan_ids = ctk.CTkButton(conn_frame, text="Поиск ID", width=90,
                                          command=self.scan_ids)
        self.btn_scan_ids.grid(row=1, column=2, padx=5, pady=5)

        self.btn_connect = ctk.CTkButton(conn_frame, text="Connect", width=100,
                                         command=self.toggle_connection)
        self.btn_connect.grid(row=1, column=3, padx=5, pady=5)

        # Глобальный фоновый опрос телеметрии - работает на всех вкладках
        self.switch_light = ctk.CTkSwitch(
            conn_frame, text=f"Лайт-опрос телеметрии ({1000 // LIGHT_POLL_MS} Гц)",
            command=self.toggle_light_poll)
        self.switch_light.select()      # по умолчанию включён
        self.switch_light.grid(row=2, column=0, columnspan=4, sticky="w", padx=10, pady=(0, 8))

        self.tabs = ctk.CTkTabview(left_panel)
        self.tabs.pack(fill="both", expand=True, padx=10, pady=5)

        tab_main = self.tabs.add("Основное")
        tab_telem = self.tabs.add("Телеметрия")
        tab_poll = self.tabs.add("Опрос")
        tab_pid = self.tabs.add("PID")
        tab_limits = self.tabs.add("Лимиты")
        tab_servo = self.tabs.add("Серво")
        tab_service = self.tabs.add("Сервис")
        tab_flash = self.tabs.add("Прошивка")

        self._build_main_tab(tab_main)
        self._build_telemetry_tab(tab_telem)
        self._build_poll_tab(tab_poll)
        self._build_pid_tab(tab_pid)
        self._build_limits_tab(tab_limits)
        self._build_servo_tab(tab_servo)
        self._build_service_tab(tab_service)
        self._build_flash_tab(tab_flash)

        # ================= ПРАВАЯ ПАНЕЛЬ =================
        right_panel = ctk.CTkFrame(self, fg_color="transparent")
        right_panel.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        right_panel.grid_rowconfigure(0, weight=5)
        right_panel.grid_rowconfigure(1, weight=3)
        right_panel.grid_columnconfigure(0, weight=1)

        self.plot_frame = ctk.CTkFrame(right_panel, corner_radius=10)
        self.plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 5))

        self.fig = Figure(figsize=(5, 4), dpi=100, facecolor='#252526')
        self.ax = self.fig.add_subplot(111)
        self.ax2 = None  # вторая ось для сигналов с другими единицами
        self._style_axes()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=(10, 0))

        ctrl = ctk.CTkFrame(self.plot_frame, fg_color="transparent")
        ctrl.pack(fill="x", padx=15, pady=(4, 0))
        ctk.CTkLabel(ctrl, text="Окно, с:", font=("Segoe UI", 11)).pack(side="left")
        self.plot_win_entry = ctk.CTkEntry(ctrl, width=60, height=24)
        self.plot_win_entry.insert(0, str(DEFAULT_PLOT_WINDOW_S))
        self.plot_win_entry.pack(side="left", padx=5)
        self._bind_enter(self.plot_win_entry, self.apply_plot_window)
        ctk.CTkButton(ctrl, text="Применить", width=90, height=24,
                      command=self.apply_plot_window).pack(side="left", padx=2)
        ctk.CTkButton(ctrl, text="Очистить", width=90, height=24,
                      command=self.clear_plot).pack(side="left", padx=2)
        self.chk_autoscroll = ctk.CTkCheckBox(ctrl, text="следовать за временем",
                                              variable=self.autoscroll_var, width=40)
        self.chk_autoscroll.pack(side="left", padx=10)
        self.chk_smooth = ctk.CTkCheckBox(ctrl, text="сглаживание", variable=self.smooth_var,
                                          width=40, command=self.redraw_plot)
        self.chk_smooth.pack(side="left", padx=5)

        self.plot_status = ctk.CTkLabel(self.plot_frame, text="График обновляется по интервалу авто-опроса",
                                        font=("Segoe UI", 10), text_color="#888888")
        self.plot_status.pack(anchor="w", padx=15, pady=(2, 8))

        term_frame = ctk.CTkFrame(right_panel, corner_radius=10)
        term_frame.grid(row=1, column=0, sticky="nsew", pady=(5, 0))
        ctk.CTkLabel(term_frame, text="ЖУРНАЛ", font=("Segoe UI", 11, "bold")).pack(
            anchor="w", padx=10, pady=5)
        self.txt_terminal = tk.Text(term_frame, bg="#1e1e1e", fg="#2ecc71", insertbackground="white",
                                    font=("Courier New", 10), bd=0, highlightthickness=0)
        self.txt_terminal.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def _build_main_tab(self, tab):
        self.btn_enable = ctk.CTkButton(tab, text="ENABLE MOTOR", state="disabled",
                                        fg_color="#2ecc71", text_color="black",
                                        hover_color="#27ae60", command=self.toggle_motor_state)
        self.btn_enable.pack(fill="x", padx=10, pady=10)

        ctk.CTkLabel(tab, text="Режим движения [0x05]:", anchor="w").pack(fill="x", padx=10, pady=(5, 2))
        self.mode_cb = ctk.CTkComboBox(tab, values=MOTION_MODES, state="disabled",
                                       command=self.change_motion_mode)
        self.mode_cb.pack(fill="x", padx=10, pady=(0, 5))

        ctk.CTkLabel(tab, text="Режим момента [0x06]:", anchor="w").pack(fill="x", padx=10, pady=(5, 2))
        self.torque_cb = ctk.CTkComboBox(tab, values=TORQUE_MODES, state="disabled",
                                         command=self.change_torque_mode)
        self.torque_cb.pack(fill="x", padx=10, pady=(0, 5))

        ctk.CTkLabel(tab, text="Уставка мотора (RAW) [0x01]:", anchor="w").pack(fill="x", padx=10, pady=(10, 2))
        tgt_row = ctk.CTkFrame(tab, fg_color="transparent")
        tgt_row.pack(fill="x", padx=10, pady=5)
        self.target_entry = ctk.CTkEntry(tgt_row, placeholder_text="0.0  (Enter — отправить)")
        self.target_entry.pack(side="left", expand=True, fill="x", padx=(0, 5))
        self._bind_enter(self.target_entry, self.send_foc_target)
        self.btn_send_target = ctk.CTkButton(tgt_row, text="Отправить", width=100, state="disabled",
                                             command=self.send_foc_target)
        self.btn_send_target.pack(side="left")

        self.btn_refresh_all = ctk.CTkButton(tab, text="Перечитать все параметры с устройства",
                                             state="disabled", command=self.refresh_all)
        self.btn_refresh_all.pack(fill="x", padx=10, pady=(20, 5))
        ctk.CTkLabel(tab, text="Выполняется автоматически при подключении.",
                     font=("Segoe UI", 10), text_color="#888888").pack(anchor="w", padx=12)

    def _build_telemetry_tab(self, tab):
        """Прямые показания без графика и без авто-опроса."""
        board = ctk.CTkFrame(tab)
        board.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(board, text="Плата [0xE4, 0xE5]",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.volt_lbl = ctk.CTkLabel(board, text="Напряжение: --- V",
                                     font=("Segoe UI", 16, "bold"), text_color="#3498db")
        self.volt_lbl.pack(anchor="w", padx=15, pady=2)
        self.temp_lbl = ctk.CTkLabel(board, text="Температура мотора: --- °C",
                                     font=("Segoe UI", 16, "bold"), text_color="#e67e22")
        self.temp_lbl.pack(anchor="w", padx=15, pady=(2, 8))

        enc = ctk.CTkFrame(tab)
        enc.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(enc, text="Энкодер MBS [0xE8..0xEB]",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.enc_angle_lbl = ctk.CTkLabel(enc, text="Угол: ---", font=("Segoe UI", 13))
        self.enc_angle_lbl.pack(anchor="w", padx=15, pady=1)
        self.enc_rpm_lbl = ctk.CTkLabel(enc, text="Скорость: ---", font=("Segoe UI", 13))
        self.enc_rpm_lbl.pack(anchor="w", padx=15, pady=1)
        self.enc_temp_lbl = ctk.CTkLabel(enc, text="Температура: ---", font=("Segoe UI", 13))
        self.enc_temp_lbl.pack(anchor="w", padx=15, pady=(1, 6))
        self.enc_status_lbl = ctk.CTkLabel(enc, text="Статус: ---", font=("Segoe UI", 13, "bold"),
                                           wraplength=400, justify="left")
        self.enc_status_lbl.pack(anchor="w", padx=15, pady=(0, 8))

        # Подписи телеметрии обновляются и кнопкой, и авто-опросом (если сигнал включён)
        self.telem_labels = {
            "voltage":   (self.volt_lbl,      "Напряжение",          "{:.2f} V"),
            "temp":      (self.temp_lbl,      "Температура мотора",  "{:.1f} °C"),
            "enc_angle": (self.enc_angle_lbl, "Угол",                "{:.4f} rad"),
            "enc_rpm":   (self.enc_rpm_lbl,   "Скорость",            "{:.3f} об/с"),
            "enc_temp":  (self.enc_temp_lbl,  "Температура",         "{:.1f} °C"),
        }

        self.btn_read_telem = ctk.CTkButton(tab, text="Прочитать телеметрию", state="disabled",
                                            command=self.read_telemetry)
        self.btn_read_telem.pack(fill="x", padx=15, pady=5)
        ctk.CTkLabel(tab, text="Читает 0xE4, 0xE5 и 0xE8..0xEB независимо от настроек авто-опроса.",
                     font=("Segoe UI", 10), text_color="#888888",
                     justify="left").pack(anchor="w", padx=15)

    def _build_poll_tab(self, tab):
        top = ctk.CTkFrame(tab)
        top.pack(fill="x", padx=10, pady=(10, 5))
        self.switch_poll = ctk.CTkSwitch(top, text="Авто-опрос", state="disabled",
                                         command=self.toggle_auto_poll)
        self.switch_poll.pack(side="left", padx=10, pady=8)
        ctk.CTkLabel(top, text="Интервал, мс:", font=("Segoe UI", 11)).pack(side="left", padx=(10, 3))
        self.poll_int_entry = ctk.CTkEntry(top, width=70)
        self.poll_int_entry.insert(0, "100")
        self.poll_int_entry.pack(side="left", padx=5)
        # Enter применяет интервал на лету, не останавливая опрос
        self._bind_enter(self.poll_int_entry, self.apply_poll_interval)

        self.poll_stat_lbl = ctk.CTkLabel(tab, text="Цикл: --- мс | запросов/с: ---",
                                          font=("Segoe UI", 10), text_color="#888888")
        self.poll_stat_lbl.pack(anchor="w", padx=15, pady=(0, 5))

        hdr = ctk.CTkFrame(tab, fg_color="transparent")
        hdr.pack(fill="x", padx=10)
        ctk.CTkLabel(hdr, text="Опрос", width=50, font=("Segoe UI", 10, "bold")).pack(side="left")
        ctk.CTkLabel(hdr, text="График", width=55, font=("Segoe UI", 10, "bold")).pack(side="left")
        ctk.CTkLabel(hdr, text="Сигнал", font=("Segoe UI", 10, "bold")).pack(side="left", padx=5)

        try:
            body = ctk.CTkScrollableFrame(tab, height=330)
        except AttributeError:      # старые версии customtkinter
            body = ctk.CTkFrame(tab)
        body.pack(fill="both", expand=True, padx=10, pady=5)

        for key, (label, reg, fmt, unit, color) in SIGNALS.items():
            row = ctk.CTkFrame(body, fg_color="transparent")
            row.pack(fill="x", pady=1)
            ctk.CTkCheckBox(row, text="", variable=self.poll_vars[key], width=40,
                            command=self.update_poll_estimate).pack(side="left")
            ctk.CTkCheckBox(row, text="", variable=self.plot_vars[key], width=45,
                            command=self._on_plot_toggle).pack(side="left")
            # регистры энкодера дороги: каждый запрос запускает транзакцию RS485
            mark = " ⚠" if key.startswith("enc_") else ""
            ctk.CTkLabel(row, text=f"{label} [0x{reg:02X}]{mark}", font=("Segoe UI", 11),
                         text_color=color, anchor="w", width=210).pack(side="left")
            val = ctk.CTkLabel(row, text="---", font=("Segoe UI", 11, "bold"), anchor="e", width=110)
            val.pack(side="right", padx=5)
            self.value_labels[key] = (val, unit)

        btns = ctk.CTkFrame(tab, fg_color="transparent")
        btns.pack(fill="x", padx=10, pady=5)
        ctk.CTkButton(btns, text="Опросить однократно", command=self.poll_once).pack(
            side="left", expand=True, fill="x", padx=2)
        ctk.CTkButton(btns, text="Очистить график", command=self.clear_plot).pack(
            side="left", expand=True, fill="x", padx=2)
        ctk.CTkLabel(tab, text="⚠ сигналы энкодера: каждый запрос запускает транзакцию RS485\n"
                              "и блокирует loop прошивки до 2 мс — опрашивайте их реже.\n"
                              "Статус энкодера и текущие показания — во вкладке «Телеметрия».",
                     font=("Segoe UI", 10), text_color="#888888",
                     justify="left").pack(anchor="w", padx=15, pady=(0, 6))

    def _build_pid_tab(self, tab):
        v = ctk.CTkFrame(tab)
        v.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(v, text="Velocity PID [0x30..0x35]", font=("Segoe UI", 11, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        vp = self.upload_velocity_pid
        self.v_p = self.create_param_entry(v, "P:", 1, 0, vp)
        self.v_i = self.create_param_entry(v, "I:", 1, 2, vp)
        self.v_d = self.create_param_entry(v, "D:", 2, 0, vp)
        self.v_ramp = self.create_param_entry(v, "Ramp:", 2, 2, vp)
        self.v_lpf = self.create_param_entry(v, "LPF Tf:", 3, 0, vp)
        self.btn_set_vpid = ctk.CTkButton(tab, text="Записать Velocity PID", state="disabled",
                                          command=self.upload_velocity_pid)
        self.btn_set_vpid.pack(fill="x", padx=15, pady=2)

        a = ctk.CTkFrame(tab)
        a.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(a, text="Angle PID [0x36..0x38]", font=("Segoe UI", 11, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        ap = self.upload_angle_pid
        self.a_p = self.create_param_entry(a, "P:", 1, 0, ap)
        self.a_i = self.create_param_entry(a, "I:", 1, 2, ap)
        self.a_d = self.create_param_entry(a, "D:", 2, 0, ap)
        self.btn_set_apid = ctk.CTkButton(tab, text="Записать Angle PID", state="disabled",
                                          command=self.upload_angle_pid)
        self.btn_set_apid.pack(fill="x", padx=15, pady=2)

    def _build_limits_tab(self, tab):
        lim = ctk.CTkFrame(tab)
        lim.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(lim, text="Ограничения [0x50..0x52]", font=("Segoe UI", 11, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        ul = self.upload_limits
        self.l_volt = self.create_param_entry(lim, "Volt Lim:", 1, 0, ul)
        self.l_curr = self.create_param_entry(lim, "Curr Lim:", 1, 2, ul)
        self.l_vel = self.create_param_entry(lim, "Vel Lim:", 2, 0, ul)
        self.btn_set_lim = ctk.CTkButton(tab, text="Записать лимиты", state="disabled",
                                         command=self.upload_limits)
        self.btn_set_lim.pack(fill="x", padx=15, pady=2)

        mot = ctk.CTkFrame(tab)
        mot.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(mot, text="Параметры мотора [0x63..0x65]", font=("Segoe UI", 11, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        um = self.upload_motor_params
        self.m_res = self.create_param_entry(mot, "R (Ohm):", 1, 0, um)
        self.m_kv = self.create_param_entry(mot, "KV:", 1, 2, um)
        self.m_pp = self.create_param_entry(mot, "Pole Pairs:", 2, 0, um)
        self.btn_set_mot = ctk.CTkButton(tab, text="Записать параметры мотора", state="disabled",
                                         command=self.upload_motor_params)
        self.btn_set_mot.pack(fill="x", padx=15, pady=2)

    def _build_servo_tab(self, tab):
        gr = ctk.CTkFrame(tab)
        gr.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(gr, text="Редукция (Gear Ratio) [0xE0]",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        gr_in = ctk.CTkFrame(gr, fg_color="transparent")
        gr_in.pack(fill="x", padx=5, pady=5)
        self.gr_entry = ctk.CTkEntry(gr_in, width=100)
        self.gr_entry.pack(side="left", padx=5)
        self._bind_enter(self.gr_entry, self.write_gear_ratio)
        self.btn_get_gr = ctk.CTkButton(gr_in, text="Прочитать", width=90, state="disabled",
                                        command=self.read_gear_ratio)
        self.btn_get_gr.pack(side="left", padx=5)
        self.btn_set_gr = ctk.CTkButton(gr_in, text="Записать", width=90, state="disabled",
                                        fg_color="#e67e22", hover_color="#d35400",
                                        command=self.write_gear_ratio)
        self.btn_set_gr.pack(side="left", padx=5)

        tgt = ctk.CTkFrame(tab)
        tgt.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(tgt, text="Целевой угол шарнира [0xE1]",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        tgt_in = ctk.CTkFrame(tgt, fg_color="transparent")
        tgt_in.pack(fill="x", padx=5, pady=5)
        self.servo_target_entry = ctk.CTkEntry(tgt_in, width=100)
        self.servo_target_entry.insert(0, "0.0")
        self.servo_target_entry.pack(side="left", padx=5)
        self._bind_enter(self.servo_target_entry, self.send_servo_target)
        self.btn_send_servo_tgt = ctk.CTkButton(tgt_in, text="Отправить угол", state="disabled",
                                                command=self.send_servo_target)
        self.btn_send_servo_tgt.pack(side="left", padx=5, expand=True, fill="x")

        ctk.CTkLabel(tab, text="Текущие угол и скорость шарнира [0xE2, 0xE3] —\n"
                              "во вкладке «Опрос» вместе с остальной телеметрией.",
                     font=("Segoe UI", 10), text_color="#888888",
                     justify="left").pack(anchor="w", padx=15, pady=10)

    def _build_service_tab(self, tab):
        canid = ctk.CTkFrame(tab)
        canid.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(canid, text="CANID узла [0xE6] (1..254)",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        row = ctk.CTkFrame(canid, fg_color="transparent")
        row.pack(fill="x", padx=5, pady=5)
        self.canid_entry = ctk.CTkEntry(row, width=80)
        self.canid_entry.pack(side="left", padx=5)
        self._bind_enter(self.canid_entry, self.write_canid)
        self.btn_get_canid = ctk.CTkButton(row, text="Прочитать", width=90, state="disabled",
                                           command=self.read_canid)
        self.btn_get_canid.pack(side="left", padx=5)
        self.btn_set_canid = ctk.CTkButton(row, text="Записать", width=90, state="disabled",
                                           fg_color="#e67e22", hover_color="#d35400",
                                           command=self.write_canid)
        self.btn_set_canid.pack(side="left", padx=5)
        ctk.CTkLabel(canid, text="Новый CANID применяется после перезагрузки",
                     font=("Segoe UI", 10), text_color="#888888").pack(anchor="w", padx=10, pady=(0, 5))

        self.btn_reboot = ctk.CTkButton(tab, text="Перезагрузить MCU [0xE7]", state="disabled",
                                        fg_color="#c0392b", hover_color="#922b21",
                                        command=self.reboot_mcu)
        self.btn_reboot.pack(fill="x", padx=15, pady=10)

        boot = ctk.CTkFrame(tab)
        boot.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(boot, text="Бутлоадер Katapult [0xEC]",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        b_in = ctk.CTkFrame(boot, fg_color="transparent")
        b_in.pack(fill="x", padx=5, pady=5)
        self.btn_check_boot = ctk.CTkButton(b_in, text="Проверить наличие", state="disabled",
                                            command=self.check_bootloader)
        self.btn_check_boot.pack(side="left", padx=5, expand=True, fill="x")
        self.btn_enter_boot = ctk.CTkButton(b_in, text="В бутлоадер", state="disabled",
                                            fg_color="#8e44ad", hover_color="#6c3483",
                                            command=self.enter_bootloader)
        self.btn_enter_boot.pack(side="left", padx=5, expand=True, fill="x")

    def _build_flash_tab(self, tab):
        ft = ctk.CTkFrame(tab)
        ft.pack(fill="x", padx=10, pady=(10, 5))
        ctk.CTkLabel(ft, text="Путь к flashtool.py",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        self.flashtool_entry = ctk.CTkEntry(ft)
        self.flashtool_entry.insert(0, DEFAULT_FLASHTOOL)
        self.flashtool_entry.pack(fill="x", padx=10, pady=5)

        uu = ctk.CTkFrame(tab)
        uu.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(uu, text="UUID устройства",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        uu_in = ctk.CTkFrame(uu, fg_color="transparent")
        uu_in.pack(fill="x", padx=5, pady=5)
        self.uuid_entry = ctk.CTkEntry(uu_in, placeholder_text="4ca84425de57")
        self.uuid_entry.pack(side="left", padx=5, expand=True, fill="x")
        self.btn_query_uuid = ctk.CTkButton(uu_in, text="Запросить (-q)", width=110,
                                            command=self.query_uuid)
        self.btn_query_uuid.pack(side="left", padx=5)

        fw = ctk.CTkFrame(tab)
        fw.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(fw, text="Файл прошивки (сборка _katapult!)",
                     font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5, 0))
        fw_in = ctk.CTkFrame(fw, fg_color="transparent")
        fw_in.pack(fill="x", padx=5, pady=5)
        self.fw_entry = ctk.CTkEntry(fw_in)
        if os.path.exists(DEFAULT_FIRMWARE):
            self.fw_entry.insert(0, DEFAULT_FIRMWARE)
        self.fw_entry.pack(side="left", padx=5, expand=True, fill="x")
        ctk.CTkButton(fw_in, text="...", width=40, command=self.browse_firmware).pack(side="left", padx=5)

        self.btn_flash = ctk.CTkButton(tab, text="ПРОШИТЬ ПО CAN", height=40, fg_color="#8e44ad",
                                       hover_color="#6c3483", font=("Segoe UI", 13, "bold"),
                                       command=self.start_flash)
        self.btn_flash.pack(fill="x", padx=15, pady=15)
        ctk.CTkLabel(tab, text="flashtool сам переведёт устройство в бутлоадер.\n"
                              "Перед прошивкой GUI отключается от шины.",
                     font=("Segoe UI", 10), text_color="#888888",
                     justify="left").pack(anchor="w", padx=15)

    def create_param_entry(self, master, label_text, row, col, on_enter=None):
        ctk.CTkLabel(master, text=label_text, font=("Segoe UI", 11)).grid(
            row=row, column=col, padx=5, pady=2, sticky="e")
        entry = ctk.CTkEntry(master, width=80, height=22)
        entry.grid(row=row, column=col + 1, padx=5, pady=2, sticky="w")
        if on_enter is not None:
            self._bind_enter(entry, on_enter)
        return entry

    @staticmethod
    def _bind_enter(entry, func):
        """Отправка значения по Enter (в т.ч. с цифровой клавиатуры)."""
        entry.bind("<Return>", lambda _e: func())
        entry.bind("<KP_Enter>", lambda _e: func())

    # ======================= СЛУЖЕБНОЕ =======================

    def log(self, text):
        self.txt_terminal.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.txt_terminal.see(tk.END)

    @staticmethod
    def _set_entry(entry, text):
        entry.delete(0, tk.END)
        entry.insert(0, text)

    def scan_can_interfaces(self):
        try:
            if os.path.exists('/sys/class/net/'):
                ifaces = [d for d in os.listdir('/sys/class/net/') if 'can' in d]
                if ifaces:
                    self.can_cb.configure(values=ifaces)
                    self.can_cb.set(ifaces[0])
                    self.log(f"Доступные интерфейсы: {', '.join(ifaces)}")
        except Exception as e:
            self.log(f"Ошибка чтения sysfs: {e}")

    def ensure_interface_up(self, channel):
        """Проверяет, поднят ли CAN-интерфейс, и при необходимости поднимает его."""
        flags_path = f"/sys/class/net/{channel}/flags"
        try:
            with open(flags_path) as f:
                if int(f.read().strip(), 16) & 0x1:  # IFF_UP
                    return True
        except (OSError, ValueError):
            self.log(f"Интерфейс {channel} не найден в системе")
            return False
        cmd = ["sudo", "-n", "ip", "link", "set", channel, "up", "type", "can",
               "bitrate", str(CAN_BITRATE)]
        self.log(f"Интерфейс {channel} выключен, поднимаю: {' '.join(cmd[2:])}")
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            self.log(f"Не удалось поднять {channel} (sudo требует пароль?).")
            self.log(f"Выполните вручную: sudo ip link set {channel} up type can bitrate {CAN_BITRATE}")
            return False
        self.log(f"Интерфейс {channel} поднят ({CAN_BITRATE} бит/с)")
        return True

    # ======================= ПОИСК ID =======================

    def scan_ids(self):
        channel = self.can_cb.get()
        if not self.ensure_interface_up(channel):
            return
        self.btn_scan_ids.configure(state="disabled", text="Поиск...")
        self.log("Поиск узлов: рассылаю запрос REG_STATUS по адресам 1..254")
        threading.Thread(target=self._scan_ids_worker, args=(channel,), daemon=True).start()

    def _scan_ids_worker(self, channel):
        try:
            found = scan_node_ids(channel)
        except Exception as e:
            self.after(0, self.log, f"Ошибка поиска: {e}")
            found = []
        finally:
            self.after(0, lambda: self.btn_scan_ids.configure(state="normal", text="Поиск ID"))

        def apply():
            if not found:
                self.log("Узлы не найдены. Проверьте питание платы, терминацию и скорость шины.")
                return
            self.log(f"Найдены узлы: {', '.join(str(a) for a in found)}")
            self.addr_cb.configure(values=[str(a) for a in found])
            if self.addr_cb.get() not in [str(a) for a in found]:
                self.addr_cb.set(str(found[0]))
        self.after(0, apply)

    # ======================= ПОДКЛЮЧЕНИЕ =======================

    def toggle_connection(self):
        if not self.running:
            channel = self.can_cb.get()
            try:
                addr = int(self.addr_cb.get())
            except ValueError:
                self.log("Ошибка: адрес узла должен быть целым числом.")
                return
            if not 1 <= addr <= 254:
                self.log("Ошибка: адрес узла вне диапазона 1..254")
                return
            if not self.ensure_interface_up(channel):
                return
            try:
                self.raw = RawRegisterClient(channel, addr)
            except Exception as e:
                self.log(f"Ошибка открытия шины: {e}")
                return
            self.running = True
            self.btn_connect.configure(text="Disconnect", fg_color="#c0392b")
            self.set_ui_state("normal")
            self.log(f"Подключено к узлу {addr} через {channel}")
            self.refresh_all()
            self.toggle_light_poll()     # поднимаем фоновый опрос, если он включён
        else:
            self.running = False
            self.auto_poll_enabled = False
            self.light_poll_active = False
            self.switch_poll.deselect()
            if self.raw:
                self.raw.close()
                self.raw = None
            self.btn_connect.configure(text="Connect", fg_color="#1f6aa5")
            self.set_ui_state("disabled")
            self.log("Связь с шиной остановлена.")

    def set_ui_state(self, state):
        for w in (self.btn_enable, self.mode_cb, self.torque_cb, self.target_entry,
                  self.btn_send_target, self.btn_refresh_all, self.switch_poll,
                  self.btn_read_telem,
                  self.btn_set_vpid, self.btn_set_apid, self.btn_set_lim, self.btn_set_mot,
                  self.btn_get_gr, self.btn_set_gr, self.btn_send_servo_tgt,
                  self.btn_get_canid, self.btn_set_canid, self.btn_reboot,
                  self.btn_check_boot, self.btn_enter_boot):
            w.configure(state=state)

    # ======================= ЧТЕНИЕ/ЗАПИСЬ =======================

    def _raw_read(self, reg, fmt='f', timeout=0.3):
        if not self.raw:
            return None
        try:
            return self.raw.read(reg, fmt, timeout)
        except Exception as e:
            self.after(0, self.log, f"Ошибка CAN при чтении 0x{reg:02X}: {e}")
            return None

    def _raw_write(self, reg, val, fmt='f'):
        if not self.raw:
            return False
        try:
            self.raw.write(reg, val, fmt)
            return True
        except Exception as e:
            self.log(f"Ошибка CAN при записи 0x{reg:02X}: {e}")
            return False

    # ---------- опрос всех параметров при подключении ----------

    def refresh_all(self):
        if not self.raw:
            return
        self.btn_refresh_all.configure(state="disabled", text="Чтение параметров...")
        threading.Thread(target=self._refresh_all_worker, daemon=True).start()

    def _refresh_all_worker(self):
        try:
            vals = {}
            plan = [
                ("enable",   REG_ENABLE,          'b'),
                ("mode",     REG_CONTROL_MODE,    'b'),
                ("torque",   REG_TORQUE_MODE,     'b'),
                ("target",   REG_TARGET,          'f'),
                ("v_p",      REG_VEL_PID_P,       'f'),
                ("v_i",      REG_VEL_PID_I,       'f'),
                ("v_d",      REG_VEL_PID_D,       'f'),
                ("v_ramp",   REG_VEL_PID_RAMP,    'f'),
                ("v_lpf",    REG_VEL_LPF_T,       'f'),
                ("a_p",      REG_ANG_PID_P,       'f'),
                ("a_i",      REG_ANG_PID_I,       'f'),
                ("a_d",      REG_ANG_PID_D,       'f'),
                ("l_volt",   REG_VOLTAGE_LIMIT,   'f'),
                ("l_curr",   REG_CURRENT_LIMIT,   'f'),
                ("l_vel",    REG_VELOCITY_LIMIT,  'f'),
                ("m_res",    REG_PHASE_RESISTANCE, 'f'),
                ("m_kv",     REG_KV,              'f'),
                ("m_pp",     REG_POLE_PAIRS,      'b'),
                ("gear",     REG_GEAR_RATIO,      'f'),
                ("canid",    REG_CANID,           'b'),
            ]
            missed = []
            for name, reg, fmt in plan:
                if not self.running:
                    return
                v = self._raw_read(reg, fmt)
                vals[name] = v
                if v is None:
                    missed.append(f"0x{reg:02X}")
            self.after(0, self._apply_refreshed, vals, missed)
        finally:
            self.after(0, lambda: self.btn_refresh_all.configure(
                state="normal" if self.running else "disabled",
                text="Перечитать все параметры с устройства"))

    def _apply_refreshed(self, v, missed):
        def num(x, digits=4):
            if x is None:
                return ""
            if x == x and abs(x - NOT_SET) < 1e-6:
                return ""          # SimpleFOC: параметр не задан
            return f"{x:.{digits}f}".rstrip('0').rstrip('.') if isinstance(x, float) else str(x)

        if v.get("enable") is not None:
            self.motor_enabled = bool(v["enable"])
            self.btn_enable.configure(
                text="DISABLE MOTOR" if self.motor_enabled else "ENABLE MOTOR",
                fg_color="#c0392b" if self.motor_enabled else "#2ecc71")
        if v.get("mode") is not None and 0 <= int(v["mode"]) < len(MOTION_MODES):
            self.mode_cb.set(MOTION_MODES[int(v["mode"])])
        if v.get("torque") is not None and 0 <= int(v["torque"]) < len(TORQUE_MODES):
            self.torque_cb.set(TORQUE_MODES[int(v["torque"])])

        for key, entry in (("target", self.target_entry), ("v_p", self.v_p), ("v_i", self.v_i),
                           ("v_d", self.v_d), ("v_ramp", self.v_ramp), ("v_lpf", self.v_lpf),
                           ("a_p", self.a_p), ("a_i", self.a_i), ("a_d", self.a_d),
                           ("l_volt", self.l_volt), ("l_curr", self.l_curr), ("l_vel", self.l_vel),
                           ("m_res", self.m_res), ("m_kv", self.m_kv), ("gear", self.gr_entry)):
            if v.get(key) is not None:
                self._set_entry(entry, num(v[key]))
        if v.get("m_pp") is not None:
            self._set_entry(self.m_pp, str(int(v["m_pp"])))
        if v.get("canid") is not None:
            self._set_entry(self.canid_entry, str(int(v["canid"])))

        if missed:
            self.log(f"Параметры прочитаны, без ответа: {', '.join(missed)}")
        else:
            self.log("Все параметры прочитаны с устройства.")

    # ======================= АВТО-ОПРОС =======================

    def _selected_signals(self):
        return [k for k in SIGNALS if self.poll_vars[k].get()]

    def apply_poll_interval(self):
        """Интервал применяется на лету: цикл опроса читает self.poll_interval
        на каждой итерации, поэтому останавливать опрос не нужно."""
        try:
            self.poll_interval = max(10, int(self.poll_int_entry.get()))
        except ValueError:
            self.log("Интервал должен быть целым числом мс")
            return
        self._set_entry(self.poll_int_entry, str(self.poll_interval))
        self.update_poll_estimate()
        self.log(f"Интервал опроса: {self.poll_interval} мс")

    def update_poll_estimate(self):
        n = len(self._selected_signals())
        try:
            interval = max(10, int(self.poll_int_entry.get()))
        except ValueError:
            interval = 100
        # запрос + ответ = 2 кадра на сигнал
        fps = n * 1000.0 / interval
        self.poll_stat_lbl.configure(
            text=f"Сигналов: {n} | расчётно {fps:.0f} запросов/с ({fps*2:.0f} кадров/с на шине)")

    def _on_plot_toggle(self):
        # график можно рисовать только по опрашиваемым данным
        for k in SIGNALS:
            if self.plot_vars[k].get() and not self.poll_vars[k].get():
                self.poll_vars[k].set(True)
        self.update_poll_estimate()
        self.redraw_plot()

    def toggle_auto_poll(self):
        if self.switch_poll.get() == 1:
            try:
                self.poll_interval = max(10, int(self.poll_int_entry.get()))
            except ValueError:
                self.poll_interval = 100
                self._set_entry(self.poll_int_entry, "100")
            if not self._selected_signals():
                self.log("Не выбрано ни одного сигнала для опроса.")
                self.switch_poll.deselect()
                return
            self.auto_poll_enabled = True
            self.poll_thread = threading.Thread(target=self._bg_poll_loop, daemon=True)
            self.poll_thread.start()
            self.log(f"Авто-опрос запущен: {len(self._selected_signals())} сигн., "
                     f"интервал {self.poll_interval} мс")
        else:
            self.auto_poll_enabled = False
            self.log("Авто-опрос остановлен.")

    def _bg_poll_loop(self):
        while self.running and self.auto_poll_enabled:
            started = time.time()
            keys = self._selected_signals()
            # таймаут на чтение не длиннее самого интервала - иначе цикл «поплывёт»
            timeout = min(0.25, max(0.02, self.poll_interval / 1000.0))
            result = {}
            for k in keys:
                if not (self.running and self.auto_poll_enabled):
                    return
                _, reg, fmt, _, _ = SIGNALS[k]
                result[k] = self._raw_read(reg, fmt, timeout)
            status = self._raw_read(REG_ENC_STATUS, 'b', timeout) if any(
                k.startswith("enc_") for k in keys) else None
            elapsed = time.time() - started
            self.after(0, self._apply_poll_result, result, status, elapsed)
            time.sleep(max(0.0, self.poll_interval / 1000.0 - elapsed))

    def poll_once(self):
        if not self.raw:
            self.log("Нет подключения.")
            return
        threading.Thread(target=self._poll_once_worker, daemon=True).start()

    def _poll_once_worker(self):
        started = time.time()
        keys = self._selected_signals()
        result = {k: self._raw_read(*SIGNALS[k][1:3]) for k in keys}
        status = self._raw_read(REG_ENC_STATUS, 'b') if any(k.startswith("enc_") for k in keys) else None
        self.after(0, self._apply_poll_result, result, status, time.time() - started)

    def _apply_poll_result(self, result, enc_status, elapsed, update_stats=True):
        t = time.time() - self.start_time
        for k, val in result.items():
            lbl, unit = self.value_labels[k]
            # getTemperature() в прошивке отдаёт -273.15 при обрыве термистора
            no_sensor = (val is not None and val == val and unit == "°C" and val <= -273.0)
            if val is None:
                text, color = "нет ответа", "#e74c3c"
            elif val != val:      # NaN - прошивка сообщает о сбое линка энкодера
                text, color = "нет RS485", "#e74c3c"
            elif no_sensor:
                text, color = "нет датчика", "#e67e22"
            else:
                text, color = f"{val:.4g} {unit}", "#dddddd"
                self.data[k].append((t, val))
            lbl.configure(text=text, text_color=color)

            # те же значения дублируем крупными подписями на вкладке «Телеметрия»
            if k in self.telem_labels:
                tl, prefix, fmt = self.telem_labels[k]
                if val is None:
                    tl.configure(text=f"{prefix}: нет ответа по CAN")
                elif val != val:
                    tl.configure(text=f"{prefix}: нет связи с энкодером (RS485)")
                elif no_sensor:
                    tl.configure(text=f"{prefix}: датчик не подключён")
                else:
                    tl.configure(text=f"{prefix}: " + fmt.format(val))

        if enc_status is not None:
            st = int(enc_status)
            if st == 0xFF:
                self.enc_status_lbl.configure(text="Статус: нет связи с энкодером (RS485)",
                                              text_color="#e74c3c")
            else:
                errs = [n for bit, n in ENC_STATUS_BITS if st & (1 << bit)]
                self.enc_status_lbl.configure(
                    text=f"Статус: 0x{st:02X} - " + (", ".join(errs) if errs else "OK"),
                    text_color="#e74c3c" if errs else "#2ecc71")

        if update_stats:
            n = len(result)
            self.poll_stat_lbl.configure(
                text=f"Сигналов: {n} | цикл {elapsed*1000:.0f} мс | "
                     f"{(n/elapsed if elapsed > 0 else 0):.0f} запросов/с")
        self.redraw_plot()

    # ---------- телеметрия по кнопке, независимо от авто-опроса ----------

    def _read_telemetry_set(self, timeout=0.3):
        """Одно чтение набора телеметрии: 0xE4, 0xE5, 0xE8..0xEB."""
        result = {k: self._raw_read(SIGNALS[k][1], SIGNALS[k][2], timeout) for k in TELEM_KEYS}
        return result, self._raw_read(REG_ENC_STATUS, 'b', timeout)

    def read_telemetry(self):
        if not self.raw:
            self.log("Нет подключения.")
            return
        self.btn_read_telem.configure(state="disabled")
        threading.Thread(target=self._read_telemetry_worker, daemon=True).start()

    def _read_telemetry_worker(self):
        try:
            result, status = self._read_telemetry_set()
            self.after(0, self._apply_poll_result, result, status, 0.0, False)
            self.after(0, self.log, "Телеметрия обновлена")
        finally:
            self.after(0, lambda: self.btn_read_telem.configure(
                state="normal" if self.running else "disabled"))

    # ---------- глобальный лайт-опрос ----------

    def toggle_light_poll(self):
        """Фоновое обновление телеметрии. Не зависит от вкладки и от
        настраиваемого авто-опроса; выключается этим же переключателем."""
        if self.switch_light.get() == 1:
            if not self.running:
                return                      # запустится сам при подключении
            if self.light_poll_active:
                return
            self.light_poll_active = True
            self.light_thread = threading.Thread(target=self._light_poll_loop, daemon=True)
            self.light_thread.start()
            self.log(f"Лайт-опрос телеметрии включён ({LIGHT_POLL_MS} мс)")
        else:
            if self.light_poll_active:
                self.light_poll_active = False
                self.log("Лайт-опрос телеметрии выключен")

    def _light_poll_loop(self):
        while self.running and self.light_poll_active:
            started = time.time()
            result, status = self._read_telemetry_set(timeout=0.15)
            if not (self.running and self.light_poll_active):
                return
            self.after(0, self._apply_poll_result, result, status, 0.0, False)
            time.sleep(max(0.0, LIGHT_POLL_MS / 1000.0 - (time.time() - started)))

    # ======================= ГРАФИК =======================

    def _style_axes(self):
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white', labelsize=8)
        self.ax.grid(True, color='#333333')
        for s in self.ax.spines.values():
            s.set_color('#555555')

    def clear_plot(self):
        for k in self.data:
            self.data[k].clear()
        self.start_time = time.time()
        self._plot_key = None      # заставит пересобрать оси
        self.redraw_plot()

    def _rebuild_plot_axes(self, selected):
        """Пересборка осей и легенды. Дорогая операция, поэтому выполняется
        только при изменении набора выбранных сигналов."""
        self.fig.clear()
        self.ax = self.fig.add_subplot(111)
        self.ax2 = None
        self.plot_lines = {}
        self.axis_of = {}
        self._style_axes()

        if not selected:
            self.ax.set_title("Выберите сигналы для графика во вкладке «Опрос»",
                              color='#888888', fontsize=9)
            self.fig.tight_layout()
            return

        # Сигналы с разными единицами разносим на две оси, иначе шкала нечитаема
        units = []
        for k in selected:
            u = SIGNALS[k][3]
            if u not in units:
                units.append(u)
        left_units, right_units = units[:1], units[1:2]
        self.axis_span = {}

        handles = []
        for k in selected:
            label, _, _, unit, color = SIGNALS[k]
            if unit in right_units:
                if self.ax2 is None:
                    self.ax2 = self.ax.twinx()
                    self.ax2.tick_params(colors='white', labelsize=8)
                ax = self.ax2
            else:
                ax = self.ax   # левая ось: первая группа единиц и все прочие
            ln, = ax.plot([], [], color=color, lw=1.6, label=f"{label}, {unit}")
            self.plot_lines[k] = ln
            self.axis_of[k] = ax
            handles.append(ln)

        self.ax.set_xlabel("время, с", color='white', fontsize=8)
        self.ax.set_ylabel(", ".join(left_units), color='white', fontsize=8)
        self.axis_span[id(self.ax)] = MIN_Y_SPAN.get(left_units[0], 0.0) if left_units else 0.0
        if self.ax2 is not None:
            self.ax2.set_ylabel(", ".join(right_units), color='white', fontsize=8)
            self.axis_span[id(self.ax2)] = MIN_Y_SPAN.get(right_units[0], 0.0)
        leg = self.ax.legend(handles=handles, labels=[h.get_label() for h in handles],
                             loc='upper left', fontsize=8, facecolor='#252526',
                             edgecolor='#555555', labelcolor='white')
        if leg:
            leg.get_frame().set_alpha(0.85)
        self.plot_status.configure(
            text=f"Внимание: выбрано {len(units)} разных единиц, читаемы только две оси"
            if len(units) > 2 else "График обновляется по интервалу авто-опроса")
        self.fig.tight_layout()

    @staticmethod
    def _smooth(ys, alpha=SMOOTH_ALPHA):
        """Экспоненциальное сглаживание для отображения. Данные в буфере
        остаются сырыми - фильтр влияет только на картинку."""
        out = []
        acc = ys[0]
        for v in ys:
            acc += alpha * (v - acc)
            out.append(acc)
        return out

    def apply_plot_window(self):
        try:
            self.plot_window_s = max(1, float(self.plot_win_entry.get()))
        except ValueError:
            self.log("Окно графика должно быть числом секунд")
            return
        self._set_entry(self.plot_win_entry, f"{self.plot_window_s:g}")
        self.redraw_plot()

    def redraw_plot(self):
        """Обновление графика. Вызывается после каждого цикла опроса, поэтому
        частота обновления равна интервалу опроса. Показывается скользящее окно
        последних plot_window_s секунд; масштаб по Y считается только по видимым
        точкам, иначе старые выбросы сплющивают текущий сигнал."""
        selected = [k for k in SIGNALS if self.plot_vars[k].get() and self.data[k]]
        key = tuple(selected)
        if key != getattr(self, "_plot_key", None):
            self._rebuild_plot_axes(selected)
            self._plot_key = key

        # правый край - последняя точка; левый зависит от режима:
        # «следовать за временем» - скользящее окно, иначе вся накопленная история
        last_t = max((self.data[k][-1][0] for k in selected if self.data[k]),
                     default=time.time() - self.start_time)
        if self.autoscroll_var.get():
            xmin = last_t - self.plot_window_s
        else:
            xmin = min((self.data[k][0][0] for k in selected if self.data[k]), default=0.0)
        now = last_t

        smooth = self.smooth_var.get()
        per_axis = {}
        for k, line in self.plot_lines.items():
            pts = [p for p in self.data[k] if p[0] >= xmin]
            ys = [p[1] for p in pts]
            if smooth and ys:
                ys = self._smooth(ys)
            line.set_data([p[0] for p in pts], ys)
            ax = self.axis_of.get(k, self.ax)
            per_axis.setdefault(id(ax), (ax, []))[1].extend(ys)

        for ax in (self.ax, self.ax2):
            if ax is not None:
                ax.set_xlim(xmin, now if now > xmin else xmin + self.plot_window_s)
        for ax, ys in per_axis.values():
            if ys:
                lo, hi = min(ys), max(ys)
                # не даём оси схлопнуться на шуме: держим минимальную высоту
                min_span = getattr(self, "axis_span", {}).get(id(ax), 0.0)
                if (hi - lo) < min_span:
                    mid = (lo + hi) / 2.0
                    lo, hi = mid - min_span / 2.0, mid + min_span / 2.0
                pad = (hi - lo) * 0.1 or 0.5
                ax.set_ylim(lo - pad, hi + pad)
        self.canvas.draw_idle()

    # ======================= УПРАВЛЕНИЕ МОТОРОМ =======================

    def toggle_motor_state(self):
        new_state = 0 if self.motor_enabled else 1
        if self._raw_write(REG_ENABLE, new_state, 'b'):
            self.motor_enabled = bool(new_state)
            self.btn_enable.configure(
                text="DISABLE MOTOR" if self.motor_enabled else "ENABLE MOTOR",
                fg_color="#c0392b" if self.motor_enabled else "#2ecc71")
            self.log(f"Мотор {'включён' if self.motor_enabled else 'выключен'}")

    def change_motion_mode(self, choice):
        val = MotionControlType[choice].value
        if self._raw_write(REG_CONTROL_MODE, val, 'b'):
            self.log(f"Режим движения: {choice} ({val})")

    def change_torque_mode(self, choice):
        val = TorqueControlType[choice].value
        if self._raw_write(REG_TORQUE_MODE, val, 'b'):
            self.log(f"Режим момента: {choice} ({val})")

    def send_foc_target(self):
        try:
            val = float(self.target_entry.get().strip())
        except ValueError:
            self.log("Ошибка: уставка должна быть числом")
            return
        if self._raw_write(REG_TARGET, val):
            self.log(f"Уставка мотора: {val}")

    def upload_velocity_pid(self):
        self._upload_group("Velocity PID", [
            (self.v_p, REG_VEL_PID_P), (self.v_i, REG_VEL_PID_I), (self.v_d, REG_VEL_PID_D),
            (self.v_ramp, REG_VEL_PID_RAMP), (self.v_lpf, REG_VEL_LPF_T)])

    def upload_angle_pid(self):
        self._upload_group("Angle PID", [
            (self.a_p, REG_ANG_PID_P), (self.a_i, REG_ANG_PID_I), (self.a_d, REG_ANG_PID_D)])

    def upload_limits(self):
        self._upload_group("Лимиты", [
            (self.l_volt, REG_VOLTAGE_LIMIT), (self.l_curr, REG_CURRENT_LIMIT),
            (self.l_vel, REG_VELOCITY_LIMIT)])

    def _upload_group(self, name, pairs):
        """Пишет только заполненные поля - пустое поле означает «не менять»."""
        written = 0
        for entry, reg in pairs:
            text = entry.get().strip()
            if not text:
                continue
            try:
                val = float(text)
            except ValueError:
                self.log(f"{name}: значение '{text}' не число, регистр 0x{reg:02X} пропущен")
                continue
            if self._raw_write(reg, val):
                written += 1
        self.log(f"{name}: записано регистров - {written}")

    def upload_motor_params(self):
        self._upload_group("Параметры мотора", [
            (self.m_res, REG_PHASE_RESISTANCE), (self.m_kv, REG_KV)])
        pp = self.m_pp.get().strip()
        if pp:
            try:
                self._raw_write(REG_POLE_PAIRS, int(pp), 'b')
                self.log(f"Pole pairs: {int(pp)}")
            except ValueError:
                self.log("Pole pairs должно быть целым числом")

    # ======================= СЕРВО =======================

    def read_gear_ratio(self):
        val = self._raw_read(REG_GEAR_RATIO)
        if val is not None:
            self._set_entry(self.gr_entry, f"{val:.4f}")
            self.log(f"Gear Ratio: {val}")
        else:
            self.log("Таймаут чтения Gear Ratio (0xE0)")

    def write_gear_ratio(self):
        try:
            val = float(self.gr_entry.get())
        except ValueError:
            self.log("Ошибка: Gear Ratio должен быть числом")
            return
        if self._raw_write(REG_GEAR_RATIO, val):
            self.log(f"Gear Ratio ({val}) отправлен")

    def send_servo_target(self):
        try:
            val = float(self.servo_target_entry.get())
        except ValueError:
            self.log("Ошибка: угол должен быть числом")
            return
        if self._raw_write(REG_SERVO_TARGET, val):
            self.log(f"Целевой угол шарнира: {val} rad")

    # ======================= СЕРВИС =======================

    def read_canid(self):
        val = self._raw_read(REG_CANID, 'b')
        if val is not None:
            self._set_entry(self.canid_entry, str(int(val)))
            self.log(f"Текущий CANID: {int(val)}")
        else:
            self.log("Таймаут чтения CANID (0xE6)")

    def write_canid(self):
        try:
            new_id = int(self.canid_entry.get())
        except ValueError:
            self.log("Ошибка: CANID должен быть целым числом 1..254")
            return
        if not 1 <= new_id <= 254:
            self.log("Ошибка: CANID вне диапазона 1..254")
            return
        if self._raw_write(REG_CANID, new_id, 'b'):
            self.log(f"CANID {new_id} записан. Применится после перезагрузки.")

    def reboot_mcu(self):
        if self._raw_write(REG_REBOOT, 1, 'b'):
            self.log("Команда перезагрузки отправлена, ответа не будет.")

    def check_bootloader(self):
        val = self._raw_read(REG_BOOTLOADER, 'b')
        if val is None:
            self.log("Таймаут чтения 0xEC")
        elif int(val) == 1:
            self.log("Katapult обнаружен во flash - прошивка по CAN доступна")
        else:
            self.log("Katapult НЕ найден - вход в бутлоадер невозможен")

    def enter_bootloader(self):
        if self._raw_write(REG_BOOTLOADER, 1, 'b'):
            self.log("Команда входа в бутлоадер отправлена. Мотор отключён.")

    # ======================= ПРОШИВКА =======================

    def browse_firmware(self):
        path = filedialog.askopenfilename(
            title="Выберите firmware.bin (сборка _katapult)",
            filetypes=[("Firmware", "*.bin"), ("Все файлы", "*.*")],
            initialdir=os.path.dirname(self.fw_entry.get()) or os.path.expanduser("~"))
        if path:
            self._set_entry(self.fw_entry, path)

    def _run_cmd_stream(self, cmd):
        self.after(0, self.log, "$ " + " ".join(cmd))
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except OSError as e:
            self.after(0, self.log, f"Не удалось запустить: {e}")
            return -1
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                self.after(0, self.log, line)
        return proc.wait()

    def _get_flashtool(self):
        path = os.path.expanduser(self.flashtool_entry.get().strip())
        if not os.path.isfile(path):
            self.log(f"flashtool.py не найден: {path}")
            return None
        return path

    def query_uuid(self):
        flashtool = self._get_flashtool()
        if not flashtool:
            return
        channel = self.can_cb.get()
        if not self.ensure_interface_up(channel):
            return
        self.btn_query_uuid.configure(state="disabled")
        threading.Thread(target=self._query_uuid_worker, args=(flashtool, channel), daemon=True).start()

    def _query_uuid_worker(self, flashtool, channel):
        try:
            cmd = [sys.executable, flashtool, "-i", channel, "-q"]
            self.after(0, self.log, "$ " + " ".join(cmd))
            try:
                out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
            except (OSError, subprocess.TimeoutExpired) as e:
                self.after(0, self.log, f"Ошибка запуска flashtool: {e}")
                return
            found = re.findall(r"Detected UUID:\s*([0-9a-fA-F]+),\s*Application:\s*(\w+)", out)
            if not found:
                self.after(0, self.log, "Устройства не найдены. Проверьте питание платы и шину.")
                return
            for uuid, app in found:
                self.after(0, self.log, f"Найдено: UUID {uuid} ({app})")
            self.after(0, self._set_entry, self.uuid_entry, found[0][0])
        finally:
            self.after(0, lambda: self.btn_query_uuid.configure(state="normal"))

    def start_flash(self):
        flashtool = self._get_flashtool()
        if not flashtool:
            return
        uuid = self.uuid_entry.get().strip().lower()
        if not re.fullmatch(r"[0-9a-f]{12}", uuid):
            self.log("Укажите UUID (12 hex-символов) - кнопка «Запросить (-q)»")
            return
        fw = os.path.expanduser(self.fw_entry.get().strip())
        if not os.path.isfile(fw):
            self.log(f"Файл прошивки не найден: {fw}")
            return
        channel = self.can_cb.get()
        if self.running:
            self.log("Отключаюсь от шины на время прошивки...")
            self.toggle_connection()
        if not self.ensure_interface_up(channel):
            return
        self.btn_flash.configure(state="disabled", text="ПРОШИВКА...")
        threading.Thread(target=self._flash_worker, args=(flashtool, channel, uuid, fw),
                         daemon=True).start()

    def _flash_worker(self, flashtool, channel, uuid, fw):
        try:
            rc = self._run_cmd_stream([sys.executable, flashtool, "-i", channel,
                                       "-u", uuid, "-f", fw])
            if rc == 0:
                self.after(0, self.log, "=== ПРОШИВКА ЗАВЕРШЕНА УСПЕШНО ===")
                self.after(0, self.log, "Устройство перезапустилось - можно подключаться (Connect).")
            else:
                self.after(0, self.log, f"=== ОШИБКА ПРОШИВКИ (код {rc}) ===")
        finally:
            self.after(0, lambda: self.btn_flash.configure(state="normal", text="ПРОШИТЬ ПО CAN"))


if __name__ == "__main__":
    app = SimpleFOCCANStudio()
    app.mainloop()
