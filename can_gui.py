#!/usr/bin/env python3
import os
import threading
import time
import tkinter as tk
import customtkinter as ctk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

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
        tab_pid = self.tabs.add("PID Тюнинг")
        tab_limits = self.tabs.add("Лимиты и Мотор")
        tab_servo = self.tabs.add("Серво / ROS") # --- НОВАЯ ВКЛАДКА ---

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
        ctk.CTkLabel(gr_frame, text="Редукция (Gear Ratio) [Регистр 0x30]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        
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
        ctk.CTkLabel(tgt_frame, text="Целевой угол шарнира [Регистр 0x31]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        
        tgt_inner = ctk.CTkFrame(tgt_frame, fg_color="transparent")
        tgt_inner.pack(fill="x", padx=5, pady=5)
        self.servo_target_entry = ctk.CTkEntry(tgt_inner, width=100)
        self.servo_target_entry.insert(0, "0.0")
        self.servo_target_entry.pack(side="left", padx=5)
        self.btn_send_servo_tgt = ctk.CTkButton(tgt_inner, text="Отправить угол", state="disabled", command=self.send_servo_target)
        self.btn_send_servo_tgt.pack(side="left", padx=5, expand=True, fill="x")

        ang_frame = ctk.CTkFrame(tab_servo)
        ang_frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(ang_frame, text="Текущий угол шарнира [Регистр 0x32]", font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=10, pady=(5,0))
        
        ang_inner = ctk.CTkFrame(ang_frame, fg_color="transparent")
        ang_inner.pack(fill="x", padx=5, pady=5)
        self.servo_angle_lbl = ctk.CTkLabel(ang_inner, text="--- rad", font=("Segoe UI", 16, "bold"), text_color="#f1c40f")
        self.servo_angle_lbl.pack(side="left", padx=15)
        self.btn_get_servo_ang = ctk.CTkButton(ang_inner, text="Запросить угол", state="disabled", command=self.read_servo_angle)
        self.btn_get_servo_ang.pack(side="right", padx=5)


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

    def toggle_connection(self):
        if not self.running:
            channel = self.can_cb.get()
            try:
                addr = int(self.addr_entry.get())
            except ValueError:
                self.log("Ошибка: Адрес узла должен быть целым числом.")
                return

            try:
                self.motors_instance = motors.can(channel=channel, target_address=addr, bitrate=1000000) 
                self.motors_instance.connect() 
                self.motor = self.motors_instance.motor(0) 

                self.motors_instance.console().subscribe(
                    on_next=lambda frame: self.after(0, self.handle_incoming_frame, frame) 
                )

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
            if self.motors_instance:
                self.motors_instance.disconnect() 
            self.btn_connect.configure(text="Connect", fg_color="#1f6aa5")
            self.set_ui_state("disabled")
            self.log("Связь с шиной остановлена.")

    def set_ui_state(self, state):
        widgets = [
            self.btn_enable, self.mode_cb, self.target_entry, self.btn_send_target,
            self.switch_poll, self.btn_set_vpid, self.btn_set_apid, self.btn_set_lim, self.btn_set_mot,
            self.btn_get_gr, self.btn_set_gr, self.btn_send_servo_tgt, self.btn_get_servo_ang
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

    # ================= КАСТОМНЫЕ РЕГИСТРЫ СЕРВО =================

    def read_gear_ratio(self):
        if not self.motor: return
        try:
            val = self.motor.get_custom(0x30, format='f')
            if val is not None:
                self.gr_entry.delete(0, tk.END)
                self.gr_entry.insert(0, f"{val:.4f}")
                self.log(f"Успешно прочитан Gear Ratio из EEPROM: {val}")
            else:
                self.log("Таймаут при чтении Gear Ratio")
        except Exception as e:
            self.log(f"Ошибка чтения: {e}")

    def write_gear_ratio(self):
        if not self.motor: return
        try:
            val = float(self.gr_entry.get())
            self.motor.set_custom(0x30, val, format='f')
            self.log(f"Gear Ratio ({val}) отправлен на контроллер для сохранения")
        except ValueError:
            self.log("Ошибка: неверный формат Gear Ratio")

    def send_servo_target(self):
        if not self.motor: return
        try:
            val = float(self.servo_target_entry.get())
            self.motor.set_custom(0x31, val, format='f')
            self.log(f"Отправлен целевой угол шарнира: {val} rad")
        except ValueError:
            self.log("Ошибка: неверный формат целевого угла")

    def read_servo_angle(self):
        if not self.motor: return
        try:
            val = self.motor.get_custom(0x32, format='f')
            if val is not None:
                self.servo_angle_lbl.configure(text=f"{val:.4f} rad")
            else:
                self.log("Таймаут при запросе угла шарнира")
        except Exception as e:
            self.log(f"Ошибка запроса угла: {e}")

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