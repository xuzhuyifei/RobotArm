import tkinter as tk
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText
import queue
import threading
import time

try:
    import serial  # type: ignore
    import serial.tools.list_ports  # type: ignore
except ModuleNotFoundError:  # pragma: no cover
    serial = None

# ==============================
# RobotArm 封装（保持不变）
# ==============================

class RobotArm:

    def __init__(self):
        self.ser = None

    def connect(self, port, baud=115200):
        if serial is None:
            raise RuntimeError("缺少依赖 pyserial")
        # timeout: 读超时；write_timeout: 写超时，避免 write() 无限阻塞
        self.ser = serial.Serial(port, baud, timeout=1, write_timeout=1)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, cmd):
        if not (self.ser and self.ser.is_open):
            raise RuntimeError("串口未连接")
        self.ser.write((cmd + "\n").encode())
        print("发送:", cmd)

    def joint(self,a1,a2,a3,s):
        self.send(f"JointAngle_{a1},{a2},{a3},{s},")

    def joint_offset(self,a1,a2,a3,s):
        self.send(f"JointAngleOffset_{a1},{a2},{a3},{s},")

    def world(self,x,y,z,s):
        self.send(f"DescartesPoint_{x},{y},{z},{s},")

    def world_offset(self,x,y,z,s):
        self.send(f"DescartesPointOffset_{x},{y},{z},{s},")

    def line(self,x,y,z,s):
        self.send(f"DescartesLine_{x},{y},{z},{s},")

    def line_offset(self,x,y,z,s):
        self.send(f"DescartesLinearOffset_{x},{y},{z},{s},")

    def suction(self,s):
        self.send(f"Suction_{s},")

    def grasp(self,s):
        self.send(f"Grasp_{s},")

    def origin(self):
        self.send("Origin")

    def stop(self):
        self.send("Stop")

    def info(self):
        self.send("Infor")


# ==============================
# GUI
# ==============================

class RobotGUI:

    def __init__(self, root: tk.Tk):
        self.root = root
        self.robot = RobotArm()
        self._pending_connect = False

        self._io_q: queue.Queue = queue.Queue()
        self._ui_q: queue.Queue = queue.Queue()
        self._io_thread = threading.Thread(target=self._io_loop, name="serial-io", daemon=True)
        self._io_thread.start()

        self.root.title("机械臂控制系统")
        self.root.geometry("900x620")
        self.root.minsize(900, 620)

        self._configure_style()
        self._build_layout()
        self.refresh_ports()
        self._set_connected(False)
        self.root.after(80, self._drain_ui_events)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _on_close(self):
        try:
            self._io_q.put_nowait(("close",))
        except Exception:
            pass
        try:
            if self.robot.ser and getattr(self.robot.ser, "is_open", False):
                try:
                    self.robot.disconnect()
                except Exception:
                    pass
        finally:
            self.root.destroy()

    def _io_loop(self):
        while True:
            task = self._io_q.get()
            if not task:
                continue
            kind = task[0]
            if kind == "close":
                try:
                    if self.robot.ser and getattr(self.robot.ser, "is_open", False):
                        self.robot.disconnect()
                except Exception:
                    pass
                return

            if kind == "connect":
                _, port, baud = task
                try:
                    # 某些异常驱动/端口 open 可能阻塞；放在后台线程，避免卡死 GUI
                    self.robot.connect(port, baud)
                except Exception as e:
                    self._ui_q.put(("connect_failed", str(e)))
                    continue
                self._ui_q.put(("connected", port, baud))
                continue

            if kind == "disconnect":
                try:
                    self.robot.disconnect()
                except Exception as e:
                    self._ui_q.put(("disconnect_failed", str(e)))
                    continue
                self._ui_q.put(("disconnected",))
                continue

            if kind == "send":
                _, cmd = task
                try:
                    self.robot.send(cmd)
                except Exception as e:
                    self._ui_q.put(("send_failed", cmd, str(e)))
                    continue
                self._ui_q.put(("sent_ok", cmd))

    def _drain_ui_events(self):
        try:
            while True:
                ev = self._ui_q.get_nowait()
                kind = ev[0]
                if kind == "connected":
                    _, port, baud = ev
                    self._pending_connect = False
                    try:
                        self.connect_btn.configure(state="normal")
                    except Exception:
                        pass
                    self._append_log("串口连接成功。")
                    self._set_connected(True)
                elif kind == "connect_failed":
                    _, err = ev
                    self._pending_connect = False
                    try:
                        self.connect_btn.configure(state="normal")
                    except Exception:
                        pass
                    self._append_log(f"串口连接失败：{err}")
                    messagebox.showerror("连接失败", f"无法连接到 {self.port_box.get()}。\n\n{err}")
                    self._set_connected(False)
                elif kind == "disconnected":
                    self._pending_connect = False
                    try:
                        self.connect_btn.configure(state="normal")
                    except Exception:
                        pass
                    self._append_log("串口已断开。")
                    self._set_connected(False)
                elif kind == "disconnect_failed":
                    _, err = ev
                    self._pending_connect = False
                    try:
                        self.connect_btn.configure(state="normal")
                    except Exception:
                        pass
                    self._append_log(f"断开失败：{err}")
                    messagebox.showerror("错误", err)
                elif kind == "send_failed":
                    _, cmd, err = ev
                    self._append_log(f"发送失败：{cmd} -> {err}")
                    messagebox.showerror("发送失败", err)
                    # 若底层提示端口已失效，UI 上也切回未连接更安全
                    if "not open" in err.lower() or "未连接" in err:
                        self._set_connected(False)
                elif kind == "sent_ok":
                    # 成功发送不额外弹窗，日志已在点击时写入
                    pass
        except queue.Empty:
            pass
        finally:
            self.root.after(80, self._drain_ui_events)

    def _configure_style(self):
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        self.root.configure(bg="#F6F7FB")

        style.configure("App.TFrame", background="#F6F7FB")
        style.configure("Card.TLabelframe", background="#FFFFFF")
        style.configure("Card.TLabelframe.Label", font=("微软雅黑", 11, "bold"), background="#FFFFFF")
        style.configure("TLabel", font=("微软雅黑", 10), background="#FFFFFF")
        style.configure("Hint.TLabel", font=("微软雅黑", 9), foreground="#666666", background="#FFFFFF")
        style.configure("TButton", font=("微软雅黑", 10))
        style.configure("Primary.TButton", font=("微软雅黑", 10, "bold"))
        style.configure("TEntry", font=("微软雅黑", 10))
        style.configure("TCombobox", font=("微软雅黑", 10))

    def _build_layout(self):
        outer = ttk.Frame(self.root, style="App.TFrame", padding=12)
        outer.pack(fill="both", expand=True)

        outer.columnconfigure(0, weight=3)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(0, weight=1)
        outer.rowconfigure(1, weight=0)

        # 左侧：控制区（标签页）
        left = ttk.Frame(outer, style="App.TFrame")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)

        self.nb = ttk.Notebook(left)
        self.nb.grid(row=0, column=0, sticky="nsew")

        self.tab_conn = ttk.Frame(self.nb, padding=10)
        self.tab_motion = ttk.Frame(self.nb, padding=10)
        self.tab_tool = ttk.Frame(self.nb, padding=10)
        self.tab_system = ttk.Frame(self.nb, padding=10)

        self.nb.add(self.tab_conn, text="连接")
        self.nb.add(self.tab_motion, text="运动")
        self.nb.add(self.tab_tool, text="末端")
        self.nb.add(self.tab_system, text="系统")

        self._build_serial_tab(self.tab_conn)
        self._build_motion_tab(self.tab_motion)
        self._build_tool_tab(self.tab_tool)
        self._build_system_tab(self.tab_system)

        # 右侧：日志与状态
        right = ttk.Labelframe(outer, text="运行日志", style="Card.TLabelframe", padding=10)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)

        self.status_badge = ttk.Label(right, text="未连接", style="Hint.TLabel")
        self.status_badge.grid(row=0, column=0, sticky="w", pady=(0, 8))

        self.log = ScrolledText(right, height=10, wrap="word", font=("Consolas", 10))
        self.log.grid(row=1, column=0, sticky="nsew")
        self.log.configure(state="disabled")

        btn_row = ttk.Frame(right, style="App.TFrame")
        btn_row.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        btn_row.columnconfigure(0, weight=1)
        ttk.Label(btn_row, text="提示：未连接时将禁用运动控制。", style="Hint.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Button(btn_row, text="清空日志", command=self._clear_log).grid(row=0, column=1, sticky="e")

        # 底部状态栏
        self.footer = ttk.Frame(outer, style="App.TFrame")
        self.footer.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        self.footer.columnconfigure(0, weight=1)
        self.footer_text = tk.StringVar(value="就绪")
        ttk.Label(self.footer, textvariable=self.footer_text, style="Hint.TLabel").grid(row=0, column=0, sticky="w")

    def _clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _append_log(self, text: str):
        self.log.configure(state="normal")
        self.log.insert("end", text.rstrip() + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _set_status(self, text: str):
        self.footer_text.set(text)

    def _set_connected(self, connected: bool):
        if connected:
            self.status_badge.configure(text=f"已连接：{self.port_box.get()} @ {self.baud_box.get()}")
            self._set_status("已连接，可以控制机械臂。")
            self.connect_btn.configure(text="断开")
        else:
            self.status_badge.configure(text="未连接")
            self._set_status("未连接，请先选择串口并连接。")
            self.connect_btn.configure(text="连接")

        state = "normal" if connected else "disabled"
        for w in self._control_widgets:
            try:
                w.configure(state=state)
            except tk.TclError:
                pass


    def _build_serial_tab(self, parent: ttk.Frame):
        frame = ttk.Labelframe(parent, text="串口设置", style="Card.TLabelframe", padding=10)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="串口").grid(row=0, column=0, sticky="w", padx=(0, 8), pady=6)
        self.port_box = ttk.Combobox(frame, width=18, state="readonly")
        self.port_box.grid(row=0, column=1, sticky="ew", pady=6)

        ttk.Button(frame, text="刷新", command=self.refresh_ports).grid(row=0, column=2, padx=(8, 0), pady=6)

        ttk.Label(frame, text="波特率").grid(row=1, column=0, sticky="w", padx=(0, 8), pady=6)
        self.baud_box = ttk.Combobox(frame, width=18, state="readonly", values=(9600, 115200))
        self.baud_box.set("115200")
        self.baud_box.grid(row=1, column=1, sticky="ew", pady=6)

        self.connect_btn = ttk.Button(frame, text="连接", style="Primary.TButton", command=self.connect_serial)
        self.connect_btn.grid(row=1, column=2, padx=(8, 0), pady=6)

        hint = ttk.Label(parent, text="连接成功后，运动/末端/系统控制会自动解锁。", style="Hint.TLabel")
        hint.grid(row=1, column=0, sticky="w", pady=(10, 0))


    def refresh_ports(self):
        if serial is None:
            self.port_box["values"] = []
            self._append_log("未安装 pyserial，无法枚举串口。")
            return

        ports = serial.tools.list_ports.comports()
        values = [p.device for p in ports]
        self.port_box["values"] = values
        if values and not self.port_box.get():
            self.port_box.set(values[0])
        self._append_log(f"刷新串口：{', '.join(values) if values else '(无)'}")


    def connect_serial(self):
        if self._pending_connect:
            return
        if self.robot.ser and self.robot.ser.is_open:
            self._pending_connect = True
            self.connect_btn.configure(state="disabled")
            self._append_log("正在断开串口...")
            self._io_q.put(("disconnect",))
            return

        port = self.port_box.get().strip()
        baud_text = self.baud_box.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口。")
            return
        if not baud_text:
            messagebox.showwarning("提示", "请选择波特率。")
            return

        try:
            baud = int(baud_text)
        except ValueError:
            messagebox.showerror("错误", "波特率必须是整数。")
            return

        self._pending_connect = True
        self.connect_btn.configure(state="disabled")
        self._append_log(f"正在连接 {port} @ {baud} ...")
        self._io_q.put(("connect", port, baud))


    def _build_motion_tab(self, parent: ttk.Frame):
        parent.columnconfigure(0, weight=1)

        self._control_widgets = []
        self._only_float_vcmd = (self.root.register(self._validate_float), "%P")

        joint = ttk.Labelframe(parent, text="关节控制", style="Card.TLabelframe", padding=10)
        joint.grid(row=0, column=0, sticky="ew")
        for c in range(8):
            joint.columnconfigure(c, weight=1 if c in (1, 3, 5, 7) else 0)

        self.j1 = self._labeled_entry(joint, "A1", 0, 0)
        self.j2 = self._labeled_entry(joint, "A2", 0, 2)
        self.j3 = self._labeled_entry(joint, "A3", 0, 4)
        self.js = self._labeled_entry(joint, "速度", 0, 6)

        btn_row = ttk.Frame(joint, style="App.TFrame")
        btn_row.grid(row=1, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        btn_row.columnconfigure(0, weight=1)

        b1 = ttk.Button(btn_row, text="关节运动", style="Primary.TButton", command=self._do_joint)
        b2 = ttk.Button(btn_row, text="关节偏移", command=self._do_joint_offset)
        b1.grid(row=0, column=0, sticky="w")
        b2.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self._control_widgets += [b1, b2]

        world = ttk.Labelframe(parent, text="世界坐标", style="Card.TLabelframe", padding=10)
        world.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for c in range(8):
            world.columnconfigure(c, weight=1 if c in (1, 3, 5, 7) else 0)

        self.wx = self._labeled_entry(world, "X", 0, 0)
        self.wy = self._labeled_entry(world, "Y", 0, 2)
        self.wz = self._labeled_entry(world, "Z", 0, 4)
        self.ws = self._labeled_entry(world, "速度", 0, 6)

        btn_row2 = ttk.Frame(world, style="App.TFrame")
        btn_row2.grid(row=1, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        btn_row2.columnconfigure(0, weight=1)

        b3 = ttk.Button(btn_row2, text="走点", style="Primary.TButton", command=self._do_world)
        b4 = ttk.Button(btn_row2, text="偏移", command=self._do_world_offset)
        b3.grid(row=0, column=0, sticky="w")
        b4.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self._control_widgets += [b3, b4]

        line = ttk.Labelframe(parent, text="直线运动", style="Card.TLabelframe", padding=10)
        line.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        for c in range(8):
            line.columnconfigure(c, weight=1 if c in (1, 3, 5, 7) else 0)

        self.lx = self._labeled_entry(line, "X", 0, 0)
        self.ly = self._labeled_entry(line, "Y", 0, 2)
        self.lz = self._labeled_entry(line, "Z", 0, 4)
        self.ls = self._labeled_entry(line, "速度", 0, 6)

        btn_row3 = ttk.Frame(line, style="App.TFrame")
        btn_row3.grid(row=1, column=0, columnspan=8, sticky="ew", pady=(10, 0))
        btn_row3.columnconfigure(0, weight=1)

        b5 = ttk.Button(btn_row3, text="直线", style="Primary.TButton", command=self._do_line)
        b6 = ttk.Button(btn_row3, text="直线偏移", command=self._do_line_offset)
        b5.grid(row=0, column=0, sticky="w")
        b6.grid(row=0, column=1, sticky="w", padx=(8, 0))
        self._control_widgets += [b5, b6]


    def _labeled_entry(self, parent: ttk.Widget, label: str, row: int, col: int) -> ttk.Entry:
        ttk.Label(parent, text=label).grid(row=row, column=col, sticky="w", padx=(0, 6), pady=4)
        e = ttk.Entry(parent, width=10, validate="key", validatecommand=self._only_float_vcmd)
        e.grid(row=row, column=col + 1, sticky="ew", pady=4)
        return e

    def _validate_float(self, proposed: str) -> bool:
        if proposed == "" or proposed == "-" or proposed == "." or proposed == "-.":
            return True
        try:
            float(proposed)
            return True
        except ValueError:
            self.root.bell()
            return False

    def _require_connected(self) -> bool:
        if self.robot.ser and self.robot.ser.is_open:
            return True
        messagebox.showwarning("提示", "请先连接串口。")
        return False

    def _do_joint(self):
        if not self._require_connected():
            return
        cmd = f"JointAngle_{self.j1.get()},{self.j2.get()},{self.j3.get()},{self.js.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    def _do_joint_offset(self):
        if not self._require_connected():
            return
        cmd = f"JointAngleOffset_{self.j1.get()},{self.j2.get()},{self.j3.get()},{self.js.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    def _do_world(self):
        if not self._require_connected():
            return
        cmd = f"DescartesPoint_{self.wx.get()},{self.wy.get()},{self.wz.get()},{self.ws.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    def _do_world_offset(self):
        if not self._require_connected():
            return
        cmd = f"DescartesPointOffset_{self.wx.get()},{self.wy.get()},{self.wz.get()},{self.ws.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    def _do_line(self):
        if not self._require_connected():
            return
        cmd = f"DescartesLine_{self.lx.get()},{self.ly.get()},{self.lz.get()},{self.ls.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    def _do_line_offset(self):
        if not self._require_connected():
            return
        cmd = f"DescartesLinearOffset_{self.lx.get()},{self.ly.get()},{self.lz.get()},{self.ls.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))


    def _build_tool_tab(self, parent: ttk.Frame):
        frame = ttk.Labelframe(parent, text="末端执行器", style="Card.TLabelframe", padding=10)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure((0, 1), weight=1)

        # 第一行：吸嘴控制
        b1 = ttk.Button(frame, text="吸嘴 ON", command=lambda: self._send_simple(f"Suction_{1},"))
        b2 = ttk.Button(frame, text="吸嘴 OFF", command=lambda: self._send_simple(f"Suction_{0},"))
        b1.grid(row=0, column=0, sticky="ew", padx=5, pady=6)
        b2.grid(row=0, column=1, sticky="ew", padx=5, pady=6)

        # 第二行：抓手控制
        b3 = ttk.Button(frame, text="抓手 ON", command=lambda: self._send_simple(f"Grasp_{1},"))
        b4 = ttk.Button(frame, text="抓手 OFF", command=lambda: self._send_simple(f"Grasp_{0},"))
        b3.grid(row=1, column=0, sticky="ew", padx=5, pady=6)
        b4.grid(row=1, column=1, sticky="ew", padx=5, pady=6)

        self._control_widgets += [b1, b2, b3, b4]


    def _build_system_tab(self, parent: ttk.Frame):
        frame = ttk.Labelframe(parent, text="系统控制", style="Card.TLabelframe", padding=10)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure((0, 1, 2), weight=1)

        b1 = ttk.Button(frame, text="回原点", style="Primary.TButton", command=lambda: self._send_simple("Origin"))
        b2 = ttk.Button(frame, text="急停", command=lambda: self._send_simple("Stop"))
        b3 = ttk.Button(frame, text="查询参数", command=lambda: self._send_simple("Infor"))

        b1.grid(row=0, column=0, sticky="ew", padx=5, pady=6)
        b2.grid(row=0, column=1, sticky="ew", padx=5, pady=6)
        b3.grid(row=0, column=2, sticky="ew", padx=5, pady=6)

        self._control_widgets += [b1, b2, b3]

    def _send_simple(self, cmd: str):
        if not self._require_connected():
            return
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))



# 主程序
if __name__ == "__main__":
    root = tk.Tk()
    if serial is None:
        messagebox.showerror(
            "缺少依赖",
            "当前环境未安装 pyserial，程序无法使用串口功能。\n\n"
            "请先安装：pip install pyserial\n"
            "或：python -m pip install pyserial",
        )
    app = RobotGUI(root)
    root.mainloop()