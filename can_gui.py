#!/usr/bin/env python3
import os
import re
import sys
import struct
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import can as pycan
from simplefoc.registers import SimpleFOCRegisters as _SFRegs

# Кастомные регистры прошивки (см. README проекта, servo_can_bridge.cpp).
# Весь обмен с ними идёт НАПРЯМУЮ через python-can (класс RawRegisterClient ниже),
# pysimplefoc используется только для штатных регистров SimpleFOC.
CUSTOM_REGS = [
    ('REG_SRV_GEAR_RATIO',  0xE0, ['f'], ['f']),
    ('REG_SRV_TARGET',      0xE1, ['f'], ['f']),
    ('REG_SRV_ANGLE',       0xE2, ['f'], []),
    ('REG_SRV_VEL',         0xE3, ['f'], []),
    ('REG_SRV_VOLTAGE',     0xE4, ['f'], []),
    ('REG_SRV_TEMPERATURE', 0xE5, ['f'], []),
    ('REG_SRV_CANID',       0xE6, ['b'], ['b']),
    ('REG_SRV_REBOOT',      0xE7, ['b'], ['b']),
    ('REG_SRV_ENC_ANGLE',   0xE8, ['f'], []),
    ('REG_SRV_ENC_RPM',     0xE9, ['f'], []),
    ('REG_SRV_ENC_TEMP',    0xEA, ['f'], []),
    ('REG_SRV_ENC_STATUS',  0xEB, ['b'], []),
    ('REG_SRV_BOOTLOADER',  0xEC, ['b'], ['b']),
]

# Регистрируем их в pysimplefoc только для того, чтобы его приёмный поток
# не сыпал "WARNING: Register id N not found" на наши ответные кадры.
for _rname, _rid, _rtypes, _wtypes in CUSTOM_REGS:
    try:
        _SFRegs.add_register(_rname, _rid, _rtypes, _wtypes)
    except Exception:
        pass

# ============ Прямой доступ к регистрам по протоколу CANCommander ============
# Формат extended (29-бит) CAN ID (см. CANCommander.h прошивки):
#   [27:20] адрес узла, [19:16] тип пакета, [15:8] номер регистра, [7:0] индекс мотора
CAN_ADDRESS_SHIFT = 20
CAN_PACKET_TYPE_SHIFT = 16
CAN_REGISTER_SHIFT = 8
PKT_READ_REQUEST = 0x1
PKT_WRITE_REQUEST = 0x2
PKT_READ_RESPONSE = 0x3


class RawRegisterClient:
    """Чтение/запись кастомных регистров сырыми CAN-кадрами (мимо pysimplefoc).

    Открывает собственный socketcan-сокет: он получает копии всех кадров шины,
    поэтому спокойно сосуществует с подключением pysimplefoc."""

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

    def write(self, reg, value, fmt='f'):
        """Запись значения в регистр (без подтверждения, как в протоколе)."""
        data = struct.pack('<f', float(value)) if fmt == 'f' else bytes([int(value) & 0xFF])
        msg = pycan.Message(arbitration_id=self._make_id(PKT_WRITE_REQUEST, reg),
                            is_extended_id=True, data=data)
        with self.lock:
            self.bus.send(msg)

    def read(self, reg, fmt='f', timeout=0.5):
        """Чтение регистра. Возвращает float/int либо None при таймауте."""
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
                data = bytes(msg.data)
                if fmt == 'f':
                    return struct.unpack('<f', data[:4])[0] if len(data) >= 4 else None
                return data[0] if data else None
# ============================================================================

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

CAN_BITRATE = 1000000
DEFAULT_FLASHTOOL = os.path.expanduser("~/katapult/scripts/flashtool.py")
DEFAULT_FIRMWARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "firmware.bin")

# Импорты SimpleFOC
from simplefoc import MotionControlType, TorqueControlType, motors
from simplefoc.registers import SimpleFOCRegisters

ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

class SimpleFOCCANStudio(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("SimpleFOC CAN Studio PRO")
        self.geometry("1200x850")

        self.motors_instance = None
        self.motor = None
        self.raw = None  # RawRegisterClient для кастомных регистров
        self.running = False
        self.motor_enabled = False
        self.auto_poll_enabled = False
        self.poll_interval = 100

        self.time_data = []
        self.telemetry_data = []
        self.start_time = time.time()

        self.create_layout()
        self.scan_can_interfaces()

    def create_layout(self):
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=4)
        self.grid_columnconfigure(1, weight=5)

        # ================= ЛЕВАЯ ПАНЕЛЬ =================
        left_panel = ctk.CTkFrame(self, corner_radius=10)
        left_panel.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        conn_frame = ctk.CTkFrame(left_panel)
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        ctk.CTkLabel(conn_frame, text="ПОДКЛЮЧЕНИЕ ШИНЫ", font=("Segoe UI", 12, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", padx=10, pady=5)
        
        self.can_cb = ctk.CTkComboBox(conn_frame, values=["can0"], width=120)
        self.can_cb.grid(row=1, column=0, padx=5, pady=5)

        self.addr_entry = ctk.CTkEntry(conn_frame, placeholder_text="ID (DEC)", width=80)
        self.addr_entry.insert(0, "228")
        self.addr_entry.grid(row=1, column=1, padx=5, pady=5)

        self.btn_connect = ctk.CTkButton(conn_frame, text="Connect", command=self.toggle_connection, width=100)
        self.btn_connect.grid(row=1, column=2, padx=5, pady=5)

        self.tabs = ctk.CTkTabview(left_panel)
        self.tabs.pack(fill="both", expand=True, padx=10, pady=5)
        
        tab_main = self.tabs.add("Основное")
        tab_pid = self.tabs.add("PID")
        tab_limits = self.tabs.add("Лимиты")
        tab_servo = self.tabs.add("Серво")
        tab_telem = self.tabs.add("Телеметрия")
        tab_service = self.tabs.add("Сервис")
        tab_flash = self.tabs.add("Прошивка")

        # --- ВКЛАДКА 1: ОСНОВНОЕ ---
        self.btn_enable = ctk.CTkButton(tab_main, text="ENABLE MOTOR", state="disabled", fg_color="#2ecc71", text_color="black", hover_color="#27ae60", command=self.toggle_motor_state)
        self.btn_enable.pack(fill="x", padx=10, pady=10)

        ctk.CTkLabel(tab_main, text="Режим работы движителя:", anchor="w").pack(fill="x", padx=10, pady=2)
        self.mode_cb = ctk.CTkComboBox(tab_main, values=["torque", "velocity", "angle", "velocity_openloop", "angle_openloop"], state="disabled", command=self.change_motor_mode)
        self.mode_cb.set("velocity")
        self.mode_cb.pack(fill="x", padx=10, pady=5)

        ctk.CTkLabel(tab_main, text="Задание уставки мотора (RAW):", anchor="w").pack(fill="x", padx=10, pady=(10,2))
        self.target_entry = ctk.CTkEntry(tab_main, placeholder_text="0.0")
        self.target_entry.insert(0, "0.0")
        self.target_entry.pack(fill="x", padx=10, pady=5)

        self.btn_send_target = ctk.CTkButton(tab_main, text="Отправить RAW Target", state="disabled", command=self.send_foc_target)
        self.btn_send_target.pack(fill="x", padx=10, pady=5)

        poll_frame = ctk.CTkFrame(tab_main)
        poll_frame.pack(fill="x", padx=10, pady=15)
        
        self.angle_label = ctk.CTkLabel(poll_frame, text="Угол мотора: --- rad", font=("Segoe UI", 16, "bold"), text_color="#3498db")
        self.angle_label.pack(pady=5)

        self.switch_poll = ctk.CTkSwitch(poll_frame, text="Авто-опрос регистра RAW угла", state="disabled", command=self.toggle_auto_poll)
        self.switch_poll.pack(pady=5)

        interval_frame = ctk.CTkFrame(poll_frame, fg_color="transparent")
        interval_frame.pack(pady=2)
        ctk.CTkLabel(interval_frame, text="Интервал (мс):", font=("Segoe UI", 10)).pack(side="left", padx=5)
        self.poll_int_entry = ctk.CTkEntry(interval_frame, width=60, height=20)
        self.poll_int_entry.insert(0, "100")
        self.poll_int_entry.pack(side="left", padx=5)

        # --- ВКЛАДКА 2: PID ТЮНИНГ ---
        v_pid_frame = ctk.CTkFrame(tab_pid)
        v_pid_frame.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(v_pid_frame, text="Velocity PID", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        self.v_p = self.create_param_entry(v_pid_frame, "P:", "0.5", 1, 0)
        self.v_i = self.create_param_entry(v_pid_frame, "I:", "10.0", 1, 2)
        self.v_d = self.create_param_entry(v_pid_frame, "D:", "0.0", 2, 0)
        self.v_ramp = self.create_param_entry(v_pid_frame, "Ramp:", "1000.0", 2, 2)
        self.btn_set_vpid = ctk.CTkButton(tab_pid, text="Записать Velocity PID", state="disabled", command=self.upload_velocity_pid)
        self.btn_set_vpid.pack(fill="x", padx=15, pady=2)

        a_pid_frame = ctk.CTkFrame(tab_pid)
        a_pid_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(a_pid_frame, text="Angle PID", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        self.a_p = self.create_param_entry(a_pid_frame, "P:", "20.0", 1, 0)
        self.a_i = self.create_param_entry(a_pid_frame, "I:", "0.0", 1, 2)
        self.a_d = self.create_param_entry(a_pid_frame, "D:", "0.0", 2, 0)
        self.btn_set_apid = ctk.CTkButton(tab_pid, text="Записать Angle PID", state="disabled", command=self.upload_angle_pid)
        self.btn_set_apid.pack(fill="x", padx=15, pady=2)

        # --- ВКЛАДКА 3: ЛИМИТЫ И ПАРАМЕТРЫ ---
        lim_frame = ctk.CTkFrame(tab_limits)
        lim_frame.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(lim_frame, text="Ограничения драйвера", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        self.l_volt = self.create_param_entry(lim_frame, "Volt Lim:", "12.0", 1, 0)
        self.l_curr = self.create_param_entry(lim_frame, "Curr Lim:", "2.0", 1, 2)
        self.l_vel = self.create_param_entry(lim_frame, "Vel Lim:", "50.0", 2, 0)
        self.btn_set_lim = ctk.CTkButton(tab_limits, text="Записать Лимиты", state="disabled", command=self.upload_limits)
        self.btn_set_lim.pack(fill="x", padx=15, pady=2)

        mot_frame = ctk.CTkFrame(tab_limits)
        mot_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(mot_frame, text="Спецификация мотора", font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=4, sticky="w", padx=5, pady=2)
        self.m_res = self.create_param_entry(mot_frame, "R (Ohm):", "12345", 1, 0) 
        self.m_kv = self.create_param_entry(mot_frame, "KV:", "12345", 1, 2)
        self.m_pp = self.create_param_entry(mot_frame, "Pole Pairs:", "7", 2, 0)
        self.btn_set_mot = ctk.CTkButton(tab_limits, text="Записать параметры мотора", state="disabled", command=self.upload_motor_params)
        self.btn_set_mot.pack(fill="x", padx=15, pady=2)

        # --- ВКЛАДКА 4: СЕРВО И ROS ---
        gr_frame = ctk.CTkFrame(tab_servo)
        gr_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(gr_frame, text="Редукция (Gear Ratio) [0xE0]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        
        gr_inner = ctk.CTkFrame(gr_frame, fg_color="transparent")
        gr_inner.pack(fill="x", padx=5, pady=5)
        self.gr_entry = ctk.CTkEntry(gr_inner, width=100)
        self.gr_entry.insert(0, "1.0")
        self.gr_entry.pack(side="left", padx=5)
        self.btn_get_gr = ctk.CTkButton(gr_inner, text="Прочитать", width=80, state="disabled", command=self.read_gear_ratio)
        self.btn_get_gr.pack(side="left", padx=5)
        self.btn_set_gr = ctk.CTkButton(gr_inner, text="Записать", width=80, state="disabled", fg_color="#e67e22", hover_color="#d35400", command=self.write_gear_ratio)
        self.btn_set_gr.pack(side="left", padx=5)

        tgt_frame = ctk.CTkFrame(tab_servo)
        tgt_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(tgt_frame, text="Целевой угол шарнира [0xE1]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        
        tgt_inner = ctk.CTkFrame(tgt_frame, fg_color="transparent")
        tgt_inner.pack(fill="x", padx=5, pady=5)
        self.servo_target_entry = ctk.CTkEntry(tgt_inner, width=100)
        self.servo_target_entry.insert(0, "0.0")
        self.servo_target_entry.pack(side="left", padx=5)
        self.btn_send_servo_tgt = ctk.CTkButton(tgt_inner, text="Отправить угол", state="disabled", command=self.send_servo_target)
        self.btn_send_servo_tgt.pack(side="left", padx=5, expand=True, fill="x")

        ang_frame = ctk.CTkFrame(tab_servo)
        ang_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(ang_frame, text="Текущий угол шарнира [0xE2]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))

        ang_inner = ctk.CTkFrame(ang_frame, fg_color="transparent")
        ang_inner.pack(fill="x", padx=5, pady=5)
        self.servo_angle_lbl = ctk.CTkLabel(ang_inner, text="--- rad", font=("Segoe UI", 16, "bold"), text_color="#f1c40f")
        self.servo_angle_lbl.pack(side="left", padx=15)
        self.btn_get_servo_ang = ctk.CTkButton(ang_inner, text="Запросить угол", state="disabled", command=self.read_servo_angle)
        self.btn_get_servo_ang.pack(side="right", padx=5)

        vel_frame = ctk.CTkFrame(tab_servo)
        vel_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(vel_frame, text="Скорость шарнира [0xE3]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))

        vel_inner = ctk.CTkFrame(vel_frame, fg_color="transparent")
        vel_inner.pack(fill="x", padx=5, pady=5)
        self.servo_vel_lbl = ctk.CTkLabel(vel_inner, text="--- rad/s", font=("Segoe UI", 16, "bold"), text_color="#f1c40f")
        self.servo_vel_lbl.pack(side="left", padx=15)
        self.btn_get_servo_vel = ctk.CTkButton(vel_inner, text="Запросить скорость", state="disabled", command=self.read_servo_vel)
        self.btn_get_servo_vel.pack(side="right", padx=5)

        # --- ВКЛАДКА 5: ТЕЛЕМЕТРИЯ (0xE4, 0xE5, 0xE8..0xEB) ---
        pwr_frame = ctk.CTkFrame(tab_telem)
        pwr_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(pwr_frame, text="Плата [0xE4, 0xE5]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        self.volt_lbl = ctk.CTkLabel(pwr_frame, text="Напряжение: --- V", font=("Segoe UI", 14, "bold"), text_color="#3498db")
        self.volt_lbl.pack(anchor="w", padx=15, pady=2)
        self.temp_lbl = ctk.CTkLabel(pwr_frame, text="Температура мотора: --- °C", font=("Segoe UI", 14, "bold"), text_color="#e67e22")
        self.temp_lbl.pack(anchor="w", padx=15, pady=(2,8))

        enc_frame = ctk.CTkFrame(tab_telem)
        enc_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(enc_frame, text="Энкодер MBS [0xE8..0xEB]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        self.enc_angle_lbl = ctk.CTkLabel(enc_frame, text="Угол: --- rad", font=("Segoe UI", 13))
        self.enc_angle_lbl.pack(anchor="w", padx=15, pady=1)
        self.enc_rpm_lbl = ctk.CTkLabel(enc_frame, text="Скорость: --- об/с", font=("Segoe UI", 13))
        self.enc_rpm_lbl.pack(anchor="w", padx=15, pady=1)
        self.enc_temp_lbl = ctk.CTkLabel(enc_frame, text="Температура: --- °C", font=("Segoe UI", 13))
        self.enc_temp_lbl.pack(anchor="w", padx=15, pady=1)
        self.enc_status_lbl = ctk.CTkLabel(enc_frame, text="Статус: ---", font=("Segoe UI", 13), wraplength=380, justify="left")
        self.enc_status_lbl.pack(anchor="w", padx=15, pady=(1,8))

        self.btn_read_telem = ctk.CTkButton(tab_telem, text="Прочитать телеметрию", state="disabled", command=self.read_telemetry)
        self.btn_read_telem.pack(fill="x", padx=15, pady=5)

        # --- ВКЛАДКА 6: СЕРВИС (0xE6, 0xE7, 0xEC) ---
        canid_frame = ctk.CTkFrame(tab_service)
        canid_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(canid_frame, text="CANID узла [0xE6] (1..254)", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        canid_inner = ctk.CTkFrame(canid_frame, fg_color="transparent")
        canid_inner.pack(fill="x", padx=5, pady=5)
        self.canid_entry = ctk.CTkEntry(canid_inner, width=80)
        self.canid_entry.pack(side="left", padx=5)
        self.btn_get_canid = ctk.CTkButton(canid_inner, text="Прочитать", width=80, state="disabled", command=self.read_canid)
        self.btn_get_canid.pack(side="left", padx=5)
        self.btn_set_canid = ctk.CTkButton(canid_inner, text="Записать", width=80, state="disabled", fg_color="#e67e22", hover_color="#d35400", command=self.write_canid)
        self.btn_set_canid.pack(side="left", padx=5)
        ctk.CTkLabel(canid_frame, text="Новый CANID применяется после перезагрузки", font=("Segoe UI", 10), text_color="#888888").pack(anchor="w", padx=10, pady=(0,5))

        self.btn_reboot = ctk.CTkButton(tab_service, text="Перезагрузить MCU [0xE7]", state="disabled", fg_color="#c0392b", hover_color="#922b21", command=self.reboot_mcu)
        self.btn_reboot.pack(fill="x", padx=15, pady=10)

        boot_frame = ctk.CTkFrame(tab_service)
        boot_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(boot_frame, text="Бутлоадер Katapult [0xEC]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        boot_inner = ctk.CTkFrame(boot_frame, fg_color="transparent")
        boot_inner.pack(fill="x", padx=5, pady=5)
        self.btn_check_boot = ctk.CTkButton(boot_inner, text="Проверить наличие", state="disabled", command=self.check_bootloader)
        self.btn_check_boot.pack(side="left", padx=5, expand=True, fill="x")
        self.btn_enter_boot = ctk.CTkButton(boot_inner, text="В бутлоадер", state="disabled", fg_color="#8e44ad", hover_color="#6c3483", command=self.enter_bootloader)
        self.btn_enter_boot.pack(side="left", padx=5, expand=True, fill="x")

        # --- ВКЛАДКА 7: ПРОШИВКА ЧЕРЕЗ KATAPULT ---
        ft_frame = ctk.CTkFrame(tab_flash)
        ft_frame.pack(fill="x", padx=10, pady=(10,5))
        ctk.CTkLabel(ft_frame, text="Путь к flashtool.py", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        self.flashtool_entry = ctk.CTkEntry(ft_frame)
        self.flashtool_entry.insert(0, DEFAULT_FLASHTOOL)
        self.flashtool_entry.pack(fill="x", padx=10, pady=5)

        uuid_frame = ctk.CTkFrame(tab_flash)
        uuid_frame.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(uuid_frame, text="UUID устройства", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        uuid_inner = ctk.CTkFrame(uuid_frame, fg_color="transparent")
        uuid_inner.pack(fill="x", padx=5, pady=5)
        self.uuid_entry = ctk.CTkEntry(uuid_inner, placeholder_text="4ca84425de57")
        self.uuid_entry.pack(side="left", padx=5, expand=True, fill="x")
        self.btn_query_uuid = ctk.CTkButton(uuid_inner, text="Запросить (-q)", width=110, command=self.query_uuid)
        self.btn_query_uuid.pack(side="left", padx=5)

        fw_frame = ctk.CTkFrame(tab_flash)
        fw_frame.pack(fill="x", padx=10, pady=5)
        ctk.CTkLabel(fw_frame, text="Файл прошивки (сборка _katapult!)", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        fw_inner = ctk.CTkFrame(fw_frame, fg_color="transparent")
        fw_inner.pack(fill="x", padx=5, pady=5)
        self.fw_entry = ctk.CTkEntry(fw_inner)
        if os.path.exists(DEFAULT_FIRMWARE):
            self.fw_entry.insert(0, DEFAULT_FIRMWARE)
        self.fw_entry.pack(side="left", padx=5, expand=True, fill="x")
        self.btn_browse_fw = ctk.CTkButton(fw_inner, text="...", width=40, command=self.browse_firmware)
        self.btn_browse_fw.pack(side="left", padx=5)

        self.btn_flash = ctk.CTkButton(tab_flash, text="ПРОШИТЬ ПО CAN", height=40, fg_color="#8e44ad", hover_color="#6c3483", font=("Segoe UI", 13, "bold"), command=self.start_flash)
        self.btn_flash.pack(fill="x", padx=15, pady=15)
        ctk.CTkLabel(tab_flash, text="flashtool сам переведёт устройство в бутлоадер.\nПеред прошивкой GUI отключается от шины.", font=("Segoe UI", 10), text_color="#888888", justify="left").pack(anchor="w", padx=15)


        # ================= ПРАВАЯ ПАНЕЛЬ =================
        right_panel = ctk.CTkFrame(self, fg_color="transparent")
        right_panel.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        right_panel.grid_rowconfigure(0, weight=5)
        right_panel.grid_rowconfigure(1, weight=4)

        self.plot_frame = ctk.CTkFrame(right_panel, corner_radius=10)
        self.plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 5))
        
        self.fig = Figure(figsize=(5, 4), dpi=100, facecolor='#252526')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#1e1e1e')
        self.ax.tick_params(colors='white')
        self.ax.grid(True, color='#333333')
        self.line, = self.ax.plot([], [], color='#2ecc71', lw=2)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

        term_frame = ctk.CTkFrame(right_panel, corner_radius=10)
        term_frame.grid(row=1, column=0, sticky="nsew", pady=(5, 0))
        ctk.CTkLabel(term_frame, text="CANDUMP / РЕАКТИВНЫЙ МОНИТОР ШИНЫ", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=5)

        self.txt_terminal = tk.Text(term_frame, bg="#1e1e1e", fg="#2ecc71", insertbackground="white", font=("Courier New", 10), bd=0, highlightthickness=0)
        self.txt_terminal.pack(fill="both", expand=True, padx=10, pady=(0, 10))

    def create_param_entry(self, master, label_text, default_val, row, col):
        ctk.CTkLabel(master, text=label_text, font=("Segoe UI", 11)).grid(row=row, column=col, padx=5, pady=2, sticky="e")
        entry = ctk.CTkEntry(master, width=75, height=22)
        entry.insert(0, default_val)
        entry.grid(row=row, column=col+1, padx=5, pady=2, sticky="w")
        return entry

    def log(self, text):
        self.txt_terminal.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {text}\n")
        self.txt_terminal.see(tk.END)

    def scan_can_interfaces(self):
        try:
            if os.path.exists('/sys/class/net/'):
                interfaces = [d for d in os.listdir('/sys/class/net/') if 'can' in d]
                if interfaces:
                    self.can_cb.configure(values=interfaces)
                    self.can_cb.set(interfaces[0])
                    self.log(f"Доступные интерфейсы: {', '.join(interfaces)}")
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
        cmd = ["sudo", "-n", "ip", "link", "set", channel, "up", "type", "can", "bitrate", str(CAN_BITRATE)]
        self.log(f"Интерфейс {channel} выключен, поднимаю: {' '.join(cmd[2:])}")
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            self.log(f"Не удалось поднять {channel} (sudo требует пароль?).")
            self.log(f"Выполните вручную: sudo ip link set {channel} up type can bitrate {CAN_BITRATE}")
            return False
        self.log(f"Интерфейс {channel} поднят ({CAN_BITRATE} бит/с)")
        return True

    def toggle_connection(self):
        if not self.running:
            channel = self.can_cb.get()
            try:
                addr = int(self.addr_entry.get())
            except ValueError:
                self.log("Ошибка: Адрес узла должен быть целым числом.")
                return

            if not self.ensure_interface_up(channel):
                return

            try:
                self.motors_instance = motors.can(channel=channel, target_address=addr, bitrate=1000000) 
                self.motors_instance.connect() 
                self.motor = self.motors_instance.motor(0) 

                self.motors_instance.console().subscribe(
                    on_next=lambda frame: self.after(0, self.handle_incoming_frame, frame) 
                )

                self.raw = RawRegisterClient(channel, addr)

                self.running = True
                self.btn_connect.configure(text="Disconnect", fg_color="#c0392b")
                self.set_ui_state("normal")
                self.log(f"Подключено к SimpleFOC Node {addr} via {channel}")
            except Exception as e:
                self.log(f"Ошибка связи: {e}")
        else:
            self.running = False
            self.auto_poll_enabled = False
            self.switch_poll.deselect()
            if self.raw:
                self.raw.close()
                self.raw = None
            if self.motors_instance:
                self.motors_instance.disconnect()
            self.btn_connect.configure(text="Connect", fg_color="#1f6aa5")
            self.set_ui_state("disabled")
            self.log("Связь с шиной остановлена.")

    def set_ui_state(self, state):
        widgets = [
            self.btn_enable, self.mode_cb, self.target_entry, self.btn_send_target,
            self.switch_poll, self.btn_set_vpid, self.btn_set_apid, self.btn_set_lim, self.btn_set_mot,
            self.btn_get_gr, self.btn_set_gr, self.btn_send_servo_tgt, self.btn_get_servo_ang,
            self.btn_get_servo_vel, self.btn_read_telem,
            self.btn_get_canid, self.btn_set_canid, self.btn_reboot,
            self.btn_check_boot, self.btn_enter_boot
        ]
        for w in widgets:
            w.configure(state=state)

    def handle_incoming_frame(self, frame):
        self.log(f"Rx Frame -> {frame}") 
        if hasattr(frame, 'register') and frame.register == SimpleFOCRegisters.REG_ANGLE: 
            if hasattr(frame, 'values') and frame.values: 
                val = frame.values if not isinstance(frame.values, list) else frame.values[0] 
                self.angle_label.configure(text=f"Угол мотора: {val:.4f} rad")
                self.update_graph(val)

    def toggle_motor_state(self):
        if not self.motor: return
        if not self.motor_enabled:
            self.motor.enable() 
            self.motor_enabled = True
            self.btn_enable.configure(text="DISABLE MOTOR", fg_color="#c0392b")
        else:
            self.motor.disable() 
            self.motor_enabled = False
            self.btn_enable.configure(text="ENABLE MOTOR", fg_color="#2ecc71")

    def change_motor_mode(self, choice):
        if not self.motor: return
        motion_mode = MotionControlType[choice] 
        self.motor.set_mode(motion_mode, TorqueControlType.voltage) 
        self.log(f"Смена режима: {motion_mode.name}")

    def send_foc_target(self):
        if not self.motor: return
        try:
            val = float(self.target_entry.get().strip())
            self.motor.set_target(val) 
        except ValueError:
            pass

    def toggle_auto_poll(self):
        if self.switch_poll.get() == 1:
            try:
                self.poll_interval = max(10, int(self.poll_int_entry.get()))
            except ValueError:
                self.poll_interval = 100
            self.auto_poll_enabled = True
            threading.Thread(target=self._bg_poll_loop, daemon=True).start()
            self.log(f"Запущен авто-опрос регистра угла ({self.poll_interval} мс)")
        else:
            self.auto_poll_enabled = False
            self.log("Авто-опрос остановлен.")

    def _bg_poll_loop(self):
        while self.running and self.auto_poll_enabled:
            if self.motor:
                try:
                    self.motor.get_angle(timeout=0.05) 
                except Exception:
                    pass
            time.sleep(self.poll_interval / 1000.0)

    # ================= КАСТОМНЫЕ РЕГИСТРЫ СЕРВО (напрямую, мимо pysimplefoc) =================

    def _raw_read(self, reg_id, fmt='f'):
        """Чтение кастомного регистра сырым CAN-кадром; None при таймауте/ошибке."""
        if not self.raw:
            return None
        try:
            return self.raw.read(reg_id, fmt)
        except Exception as e:
            self.log(f"Ошибка CAN при чтении 0x{reg_id:02X}: {e}")
            return None

    def _raw_write(self, reg_id, val, fmt='f'):
        if not self.raw:
            return False
        try:
            self.raw.write(reg_id, val, fmt)
            return True
        except Exception as e:
            self.log(f"Ошибка CAN при записи 0x{reg_id:02X}: {e}")
            return False

    def read_gear_ratio(self):
        val = self._raw_read(0xE0)
        if val is not None:
            self.gr_entry.delete(0, tk.END)
            self.gr_entry.insert(0, f"{val:.4f}")
            self.log(f"Gear Ratio прочитан: {val}")
        else:
            self.log("Таймаут чтения Gear Ratio (0xE0)")

    def write_gear_ratio(self):
        try:
            val = float(self.gr_entry.get())
        except ValueError:
            self.log("Ошибка: Gear Ratio должен быть числом")
            return
        if self._raw_write(0xE0, val):
            self.log(f"Gear Ratio ({val}) отправлен")

    def send_servo_target(self):
        try:
            val = float(self.servo_target_entry.get())
        except ValueError:
            self.log("Ошибка: угол должен быть числом")
            return
        if self._raw_write(0xE1, val):
            self.log(f"Целевой угол отправлен: {val} rad")

    def read_servo_angle(self):
        val = self._raw_read(0xE2)
        if val is not None:
            self.servo_angle_lbl.configure(text=f"{val:.4f} rad")
        else:
            self.log("Таймаут запроса угла (0xE2)")

    def read_servo_vel(self):
        val = self._raw_read(0xE3)
        if val is not None:
            self.servo_vel_lbl.configure(text=f"{val:.4f} rad/s")
        else:
            self.log("Таймаут чтения скорости шарнира (0xE3)")

    def _decode_enc_status(self, st):
        errs = [name for bit, name in ENC_STATUS_BITS if st & (1 << bit)]
        return ", ".join(errs) if errs else "OK"

    def _fmt_enc(self, val, fmt_str):
        """NaN от прошивки = сбой RS485-линка с энкодером."""
        if val is None:
            return "нет ответа по CAN"
        if val != val:  # NaN
            return "нет связи с энкодером (RS485)"
        return fmt_str.format(val)

    def read_telemetry(self):
        val = self._raw_read(0xE4)
        if val is not None: self.volt_lbl.configure(text=f"Напряжение: {val:.2f} V")
        val = self._raw_read(0xE5)
        if val is not None: self.temp_lbl.configure(text=f"Температура мотора: {val:.1f} °C")
        self.enc_angle_lbl.configure(text="Угол: " + self._fmt_enc(self._raw_read(0xE8), "{:.4f} rad"))
        self.enc_rpm_lbl.configure(text="Скорость: " + self._fmt_enc(self._raw_read(0xE9), "{:.3f} об/с"))
        self.enc_temp_lbl.configure(text="Температура: " + self._fmt_enc(self._raw_read(0xEA), "{:.1f} °C"))
        val = self._raw_read(0xEB, 'b')
        if val is None:
            self.enc_status_lbl.configure(text="Статус: нет ответа по CAN")
        elif int(val) == 0xFF:
            self.enc_status_lbl.configure(text="Статус: нет связи с энкодером (RS485)")
        else:
            self.enc_status_lbl.configure(text=f"Статус: 0x{int(val):02X} - {self._decode_enc_status(int(val))}")
        self.log("Телеметрия обновлена")

    # ================= СЕРВИС (0xE6, 0xE7, 0xEC) =================

    def read_canid(self):
        val = self._raw_read(0xE6, 'b')
        if val is not None:
            self.canid_entry.delete(0, tk.END)
            self.canid_entry.insert(0, str(int(val)))
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
        if self._raw_write(0xE6, new_id, 'b'):
            self.log(f"CANID {new_id} записан. Применится после перезагрузки (кнопка Reboot).")

    def reboot_mcu(self):
        if self._raw_write(0xE7, 1, 'b'):
            self.log("Команда перезагрузки отправлена. Устройство перезапускается, ответа не будет.")

    def check_bootloader(self):
        val = self._raw_read(0xEC, 'b')
        if val is None:
            self.log("Таймаут чтения 0xEC")
        elif int(val) == 1:
            self.log("Katapult обнаружен во flash - прошивка по CAN доступна")
        else:
            self.log("Katapult НЕ найден (прошивка без бутлоадера) - вход в бутлоадер невозможен")

    def enter_bootloader(self):
        if self._raw_write(0xEC, 1, 'b'):
            self.log("Команда входа в бутлоадер отправлена. Мотор отключён, устройство в Katapult.")

    # ================= ПРОШИВКА ЧЕРЕЗ KATAPULT =================

    def browse_firmware(self):
        path = filedialog.askopenfilename(
            title="Выберите firmware.bin (сборка _katapult)",
            filetypes=[("Firmware", "*.bin"), ("Все файлы", "*.*")],
            initialdir=os.path.dirname(self.fw_entry.get()) or os.path.expanduser("~"))
        if path:
            self.fw_entry.delete(0, tk.END)
            self.fw_entry.insert(0, path)

    def _run_cmd_stream(self, cmd):
        """Запуск команды с трансляцией вывода в терминал GUI. Возвращает код возврата."""
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
        if not flashtool: return
        channel = self.can_cb.get()
        if not self.ensure_interface_up(channel): return
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
            uuid = found[0][0]
            def fill():
                self.uuid_entry.delete(0, tk.END)
                self.uuid_entry.insert(0, uuid)
            self.after(0, fill)
        finally:
            self.after(0, lambda: self.btn_query_uuid.configure(state="normal"))

    def start_flash(self):
        flashtool = self._get_flashtool()
        if not flashtool: return
        uuid = self.uuid_entry.get().strip().lower()
        if not re.fullmatch(r"[0-9a-f]{12}", uuid):
            self.log("Укажите UUID (12 hex-символов) - кнопка 'Запросить (-q)'")
            return
        fw = os.path.expanduser(self.fw_entry.get().strip())
        if not os.path.isfile(fw):
            self.log(f"Файл прошивки не найден: {fw}")
            return
        channel = self.can_cb.get()
        # На время прошивки освобождаем шину от нашего трафика
        if self.running:
            self.log("Отключаюсь от шины на время прошивки...")
            self.toggle_connection()
        if not self.ensure_interface_up(channel): return
        self.btn_flash.configure(state="disabled", text="ПРОШИВКА...")
        threading.Thread(target=self._flash_worker, args=(flashtool, channel, uuid, fw), daemon=True).start()

    def _flash_worker(self, flashtool, channel, uuid, fw):
        try:
            rc = self._run_cmd_stream([sys.executable, flashtool, "-i", channel, "-u", uuid, "-f", fw])
            if rc == 0:
                self.after(0, self.log, "=== ПРОШИВКА ЗАВЕРШЕНА УСПЕШНО ===")
                self.after(0, self.log, "Устройство перезапустилось - можно подключаться (Connect).")
            else:
                self.after(0, self.log, f"=== ОШИБКА ПРОШИВКИ (код {rc}) ===")
        finally:
            self.after(0, lambda: self.btn_flash.configure(state="normal", text="ПРОШИТЬ ПО CAN"))

    # ============================================================

    def upload_velocity_pid(self):
        if not self.motor: return
        try:
            p = float(self.v_p.get())
            i = float(self.v_i.get())
            data_d = float(self.v_d.get())
            ramp = float(self.v_ramp.get())
            self.motor.set_velocity_pid(p=p, i=i, d=data_d, ramp=ramp) 
            self.log(f"Записан Vel PID: P={p}, I={i}, D={data_d}, Ramp={ramp}")
        except ValueError:
            self.log("Ошибка: Заполните все поля числами (float)")

    def upload_angle_pid(self):
        if not self.motor: return
        try:
            p = float(self.a_p.get())
            i = float(self.a_i.get())
            data_d = float(self.a_d.get())
            self.motor.set_angle_pid(p=p, i=i, d=data_d) 
            self.log(f"Записан Ang PID: P={p}, I={i}, D={data_d}")
        except ValueError:
            self.log("Ошибка: Заполните все поля числами (float)")

    def upload_limits(self):
        if not self.motor: return
        try:
            v = float(self.l_volt.get())
            c = float(self.l_curr.get())
            vel = float(self.l_vel.get())
            self.motor.set_limits(max_voltage=v, max_current=c, max_velocity=vel) 
        except ValueError:
            self.log("Ошибка: Заполните лимиты корректно")

    def upload_motor_params(self):
        if not self.motor: return
        try:
            r = float(self.m_res.get())
            kv = float(self.m_kv.get())
            pp = int(self.m_pp.get())
            self.motor.set_motor_parameters(resistance=r, kv=kv, pole_pairs=pp) 
        except ValueError:
            self.log("Ошибка: Неверный формат параметров")

    def update_graph(self, next_val):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.telemetry_data.append(next_val)

        if len(self.time_data) > 70:
            self.time_data.pop(0)
            self.telemetry_data.pop(0)

        self.line.set_data(self.time_data, self.telemetry_data)
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

if __name__ == "__main__":
    app = SimpleFOCCANStudio()
    app.mainloop()