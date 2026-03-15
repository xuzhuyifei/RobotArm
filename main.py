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

        # 步骤缓冲区（本地镜像）
        self.recording_steps = False
        self.step_records = []  # 每个元素：{"index": int, "type": int, "desc": str, "cmd": str}
        
        # 步骤参数设置 - 用于 SetStep 指令的默认参数
        self.step_delay = tk.StringVar(value="0")  # 延时 ms
        self.step_io_pin = tk.StringVar(value="0")  # IO 引脚
        self.step_io_state = tk.StringVar(value="0")  # IO 状态
        
        # 插入模式标志
        self.insert_mode = False  # 是否处于插入模式
        self.insert_position = None  # 插入位置（选中的步骤索引）

        self._io_q: queue.Queue = queue.Queue()
        self._ui_q: queue.Queue = queue.Queue()
        self._io_thread = threading.Thread(target=self._io_loop, name="serial-io", daemon=True)
        self._io_thread.start()

        self.root.title("机械臂控制系统")
        self.root.geometry("1100x640")
        self.root.minsize(1000, 600)

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
        outer.rowconfigure(0, weight=1)
        outer.rowconfigure(1, weight=0)
        outer.columnconfigure(0, weight=1)

        # 主区域：PanedWindow 可拖动调整宽度
        paned = ttk.PanedWindow(outer, orient="horizontal")
        paned.grid(row=0, column=0, sticky="nsew")
        paned.columnconfigure(0, weight=1)

        # 左侧：控制区（标签页）
        left = ttk.Frame(paned, style="App.TFrame")
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)
        paned.add(left, weight=3)

        self.nb = ttk.Notebook(left)
        self.nb.grid(row=0, column=0, sticky="nsew")

        self.tab_conn = ttk.Frame(self.nb, padding=10)
        self.tab_motion = ttk.Frame(self.nb, padding=10)
        self.tab_tool = ttk.Frame(self.nb, padding=10)
        self.tab_program = ttk.Frame(self.nb, padding=10)
        self.tab_system = ttk.Frame(self.nb, padding=10)

        self.nb.add(self.tab_conn, text="连接")
        self.nb.add(self.tab_motion, text="运动")
        self.nb.add(self.tab_tool, text="末端")
        self.nb.add(self.tab_program, text="程序")
        self.nb.add(self.tab_system, text="系统")

        self._build_serial_tab(self.tab_conn)
        self._build_motion_tab(self.tab_motion)
        self._build_tool_tab(self.tab_tool)
        self._build_program_tab(self.tab_program)
        self._build_system_tab(self.tab_system)

        # 中间：步骤缓冲区（始终显示）
        frm_buf = self._build_step_buffer_panel(paned)
        paned.add(frm_buf, weight=2)

        # 右侧：运行日志（宽度略大，可拖动调整）
        right = ttk.Labelframe(paned, text="运行日志", style="Card.TLabelframe", padding=10)
        right.rowconfigure(1, weight=1)
        right.columnconfigure(0, weight=1)
        paned.add(right, weight=3)

        self.status_badge = ttk.Label(right, text="未连接", style="Hint.TLabel")
        self.status_badge.grid(row=0, column=0, sticky="w", pady=(0, 8))

        self.log = ScrolledText(right, height=12, wrap="word", font=("Consolas", 10))
        self.log.grid(row=1, column=0, sticky="nsew")
        self.log.configure(state="disabled")

        btn_row = ttk.Frame(right, style="App.TFrame")
        btn_row.grid(row=2, column=0, sticky="ew", pady=(8, 0))
        btn_row.columnconfigure(0, weight=1)
        ttk.Label(btn_row, text="提示：未连接时将禁用运动控制。", style="Hint.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Button(btn_row, text="清空日志", command=self._clear_log).grid(row=0, column=1, sticky="e")

        # 底部状态栏
        self.footer = ttk.Frame(outer, style="App.TFrame")
        self.footer.grid(row=1, column=0, sticky="ew", pady=(10, 0))
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
        if self.recording_steps:
            self._record_step(
                type_code=1,
                p1=self.j1.get(),
                p2=self.j2.get(),
                p3=self.j3.get(),
                speed=self.js.get(),
                desc=f"关节 {self.j1.get()},{self.j2.get()},{self.j3.get()} v={self.js.get()}",
            )

    def _do_joint_offset(self):
        if not self._require_connected():
            return
        cmd = f"JointAngleOffset_{self.j1.get()},{self.j2.get()},{self.j3.get()},{self.js.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))
        if self.recording_steps:
            self._record_step(
                type_code=2,
                p1=self.j1.get(),
                p2=self.j2.get(),
                p3=self.j3.get(),
                speed=self.js.get(),
                desc=f"关节偏移 {self.j1.get()},{self.j2.get()},{self.j3.get()} v={self.js.get()}",
            )

    def _do_world(self):
        if not self._require_connected():
            return
        cmd = f"DescartesPoint_{self.wx.get()},{self.wy.get()},{self.wz.get()},{self.ws.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))
        if self.recording_steps:
            self._record_step(
                type_code=3,
                p1=self.wx.get(),
                p2=self.wy.get(),
                p3=self.wz.get(),
                speed=self.ws.get(),
                desc=f"走点 X={self.wx.get()},Y={self.wy.get()},Z={self.wz.get()} v={self.ws.get()}",
            )

    def _do_world_offset(self):
        if not self._require_connected():
            return
        cmd = f"DescartesPointOffset_{self.wx.get()},{self.wy.get()},{self.wz.get()},{self.ws.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))
        if self.recording_steps:
            self._record_step(
                type_code=4,
                p1=self.wx.get(),
                p2=self.wy.get(),
                p3=self.wz.get(),
                speed=self.ws.get(),
                desc=f"点偏移 X={self.wx.get()},Y={self.wy.get()},Z={self.wz.get()} v={self.ws.get()}",
            )

    def _do_line(self):
        if not self._require_connected():
            return
        cmd = f"DescartesLine_{self.lx.get()},{self.ly.get()},{self.lz.get()},{self.ls.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))
        if self.recording_steps:
            self._record_step(
                type_code=5,
                p1=self.lx.get(),
                p2=self.ly.get(),
                p3=self.lz.get(),
                speed=self.ls.get(),
                desc=f"直线 X={self.lx.get()},Y={self.ly.get()},Z={self.lz.get()} v={self.ls.get()}",
            )

    def _do_line_offset(self):
        if not self._require_connected():
            return
        cmd = f"DescartesLinearOffset_{self.lx.get()},{self.ly.get()},{self.lz.get()},{self.ls.get()},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))
        if self.recording_steps:
            self._record_step(
                type_code=6,
                p1=self.lx.get(),
                p2=self.ly.get(),
                p3=self.lz.get(),
                speed=self.ls.get(),
                desc=f"直线偏移 X={self.lx.get()},Y={self.ly.get()},Z={self.lz.get()} v={self.ls.get()}",
            )


    def _build_tool_tab(self, parent: ttk.Frame):
        frame = ttk.Labelframe(parent, text="末端执行器", style="Card.TLabelframe", padding=10)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure((0, 1, 2), weight=1)

        # 第一行：吸嘴控制
        ttk.Label(frame, text="吸嘴:").grid(row=0, column=0, sticky="w", padx=5, pady=6)
        self.suction_var = tk.StringVar(value="0")
        self.suction_combo = ttk.Combobox(frame, textvariable=self.suction_var, width=8, state="readonly", values=("0", "1"))
        self.suction_combo.grid(row=0, column=1, sticky="w", pady=6)
        b1 = ttk.Button(frame, text="执行", command=self._do_suction)
        b1.grid(row=0, column=2, sticky="ew", padx=5, pady=6)

        # 第二行：抓手控制
        ttk.Label(frame, text="抓手:").grid(row=1, column=0, sticky="w", padx=5, pady=6)
        self.grasp_var = tk.StringVar(value="0")
        self.grasp_combo = ttk.Combobox(frame, textvariable=self.grasp_var, width=8, state="readonly", values=("0", "1"))
        self.grasp_combo.grid(row=1, column=1, sticky="w", pady=6)
        b3 = ttk.Button(frame, text="执行", command=self._do_grasp)
        b3.grid(row=1, column=2, sticky="ew", padx=5, pady=6)

        self._control_widgets += [b1, b3]

    def _do_suction(self):
        """执行吸嘴控制"""
        if not self._require_connected():
            return
        state = self.suction_var.get()
        cmd = f"Suction_{state},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    def _do_grasp(self):
        """执行抓手控制"""
        if not self._require_connected():
            return
        state = self.grasp_var.get()
        cmd = f"Grasp_{state},"
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))


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

    def _build_step_buffer_panel(self, parent: ttk.Widget) -> ttk.Frame:
        """步骤缓冲区面板，始终显示在右侧，独立于程序标签"""
        frm_buf = ttk.Labelframe(parent, text="步骤缓冲区", style="Card.TLabelframe", padding=10)
        frm_buf.columnconfigure(0, weight=1)
        frm_buf.rowconfigure(0, weight=1)

        columns = ("step", "desc")
        self.step_tree = ttk.Treeview(frm_buf, columns=columns, show="headings", height=8)
        self.step_tree.heading("step", text="步骤")
        self.step_tree.heading("desc", text="指令")
        self.step_tree.column("step", width=60, anchor="center")
        self.step_tree.column("desc", anchor="w")
        self.step_tree.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(frm_buf, orient="vertical", command=self.step_tree.yview)
        self.step_tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.grid(row=0, column=1, sticky="ns")

        buf_btns = ttk.Frame(frm_buf, style="App.TFrame")
        buf_btns.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        buf_btns.columnconfigure((0, 1, 2, 3), weight=1)

        self.record_btn = ttk.Button(buf_btns, text="开始录制步骤", command=self._toggle_record)
        btn_insert_empty = ttk.Button(buf_btns, text="插入空步骤", command=self._insert_empty_step)
        btn_delete = ttk.Button(buf_btns, text="删除步骤", command=self._delete_step)
        btn_clear = ttk.Button(buf_btns, text="清空本地显示", command=self._clear_local_steps)

        self.record_btn.grid(row=0, column=0, sticky="ew", padx=4)
        btn_insert_empty.grid(row=0, column=1, sticky="ew", padx=4)
        btn_delete.grid(row=0, column=2, sticky="ew", padx=4)
        btn_clear.grid(row=0, column=3, sticky="ew", padx=4)

        # 步骤参数设置区域（用于 SetStep 指令的默认参数）
        frm_params = ttk.Frame(frm_buf, style="App.TFrame")
        frm_params.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        frm_params.columnconfigure((0, 2, 4), weight=0)
        frm_params.columnconfigure((1, 3, 5), weight=1)
        
        ttk.Label(frm_params, text="默认延时 (ms):", style="Hint.TLabel").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.step_delay_entry = ttk.Entry(frm_params, textvariable=self.step_delay, width=8)
        self.step_delay_entry.grid(row=0, column=1, sticky="w", pady=4)
        
        ttk.Label(frm_params, text="IO 引脚:", style="Hint.TLabel").grid(row=0, column=2, sticky="w", padx=(8, 4))
        self.step_io_pin_entry = ttk.Entry(frm_params, textvariable=self.step_io_pin, width=8)
        self.step_io_pin_entry.grid(row=0, column=3, sticky="w", pady=4)
        
        ttk.Label(frm_params, text="IO 状态:", style="Hint.TLabel").grid(row=0, column=4, sticky="w", padx=(8, 4))
        self.step_io_state_entry = ttk.Entry(frm_params, textvariable=self.step_io_state, width=8)
        self.step_io_state_entry.grid(row=0, column=5, sticky="w", pady=4)
        
        # 提示信息
        hint_label = ttk.Label(frm_params, text="💡 选中步骤后录制将插入到其后；选中空步骤则替换", style="Hint.TLabel")
        hint_label.grid(row=3, column=0, columnspan=6, sticky="w", pady=(4, 0))

        self._control_widgets += [self.record_btn, btn_insert_empty, btn_delete, btn_clear, 
                                  self.step_delay_entry, self.step_io_pin_entry, self.step_io_state_entry]
        return frm_buf

    def _build_program_tab(self, parent: ttk.Frame):
        parent.columnconfigure(0, weight=1)

        # 配方管理
        frm_top = ttk.Labelframe(parent, text="配方管理", style="Card.TLabelframe", padding=10)
        frm_top.grid(row=0, column=0, sticky="ew")
        frm_top.columnconfigure((1, 2, 3), weight=1)

        ttk.Label(frm_top, text="配方号").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
        self.recipe_var = tk.StringVar(value="1")
        self.recipe_box = ttk.Combobox(frm_top, textvariable=self.recipe_var, width=8, state="readonly",
                                       values=("1", "2", "3"))
        self.recipe_box.grid(row=0, column=1, sticky="w", pady=4)

        btn_set_ok = ttk.Button(frm_top, text="写入配方 (SetStepOK)",
                                command=self._do_set_step_ok)
        btn_clean = ttk.Button(frm_top, text="清除配方 (Clean)",
                               command=self._do_clean_recipe)
        btn_edit = ttk.Button(frm_top, text="编辑配方 (Editor)",
                              command=self._do_edit_recipe)
        btn_set_ok.grid(row=1, column=0, sticky="ew", padx=4, pady=4, columnspan=1)
        btn_clean.grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        btn_edit.grid(row=1, column=2, sticky="ew", padx=4, pady=4)

        self._control_widgets += [btn_set_ok, btn_clean, btn_edit]

        frm_run = ttk.Labelframe(parent, text="运行控制", style="Card.TLabelframe", padding=10)
        frm_run.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        frm_run.columnconfigure((1, 2), weight=1)

        ttk.Label(frm_run, text="运行配方").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
        self.run_recipe_var = tk.StringVar(value="1")
        self.run_recipe_box = ttk.Combobox(frm_run, textvariable=self.run_recipe_var, width=8, state="readonly",
                                           values=("1", "2", "3"))
        self.run_recipe_box.grid(row=0, column=1, sticky="w", pady=4)

        btn_single = ttk.Button(frm_run, text="单循环 (Single)",
                                command=self._do_single_run)
        btn_cycle = ttk.Button(frm_run, text="循环运行 (Cycle)",
                               command=self._do_cycle_run)
        btn_refer = ttk.Button(frm_run, text="查询配方 (Refer)",
                               command=self._do_refer)
        btn_single.grid(row=1, column=0, sticky="ew", padx=4, pady=4)
        btn_cycle.grid(row=1, column=1, sticky="ew", padx=4, pady=4)
        btn_refer.grid(row=1, column=2, sticky="ew", padx=4, pady=4)

        self._control_widgets += [btn_single, btn_cycle, btn_refer]

        frm_adv = ttk.Labelframe(parent, text="高级设置", style="Card.TLabelframe", padding=10)
        frm_adv.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        frm_adv.columnconfigure((1, 3), weight=1)

        ttk.Label(frm_adv, text="自动校准次数 Align").grid(row=0, column=0, sticky="w", padx=(0, 6), pady=4)
        self.align_entry = ttk.Entry(frm_adv, width=8, validate="key", validatecommand=self._only_float_vcmd)
        self.align_entry.insert(0, "0")
        self.align_entry.grid(row=0, column=1, sticky="w", pady=4)

        btn_align = ttk.Button(frm_adv, text="设置自动校准", command=self._do_align)
        btn_trigger = ttk.Button(frm_adv, text="DI触发循环 (Trigger)", command=self._do_trigger)
        btn_align.grid(row=0, column=2, sticky="ew", padx=4, pady=4)
        btn_trigger.grid(row=0, column=3, sticky="ew", padx=4, pady=4)

        self._control_widgets += [btn_align, btn_trigger]

    def _send_simple(self, cmd: str):
        if not self._require_connected():
            return
        self._append_log(f"发送：{cmd}")
        self._io_q.put(("send", cmd))

    # ===== 步骤缓冲区：本地镜像与写入 =====

    def _toggle_record(self):
        if not self._require_connected():
            return
        self.recording_steps = not self.recording_steps
        if self.recording_steps:
            self.record_btn.configure(text="停止录制步骤")
            self._append_log("开始录制步骤：之后的运动指令会同时写入步骤缓冲区 (SetStep)。")
            self._append_log("提示：选中某个步骤后，新指令将插入到该步骤之后；若选中空指令则直接替换。")
        else:
            self.record_btn.configure(text="开始录制步骤")
            self._append_log("停止录制步骤：之后的运动指令仅发送本身，不再写入步骤缓冲区。")
            # 退出插入模式
            self.insert_mode = False
            self.insert_position = None

    def _record_step(self, type_code: int, p1: str, p2: str, p3: str, speed: str, desc: str, delay: str = None, io_pin: str = None, io_state: str = None):
        """录制步骤到缓冲区
        
        Args:
            type_code: 指令类型 (1-关节坐标 2-关节偏移 3-世界坐标 4-世界偏移 5-直线 6-直线偏移 7-DI 8-DO 9-外设)
            p1, p2, p3: 主要参数 (角度/坐标等)
            speed: 速度
            desc: 描述文本
            delay: 延时 (ms)，默认从 UI 获取
            io_pin: IO 引脚，默认从 UI 获取
            io_state: IO 状态，默认从 UI 获取
        """
        if not self.recording_steps:
            return
        try:
            execute_flag = 1  # 保存并执行
            
            # 使用 UI 设置的默认值，除非调用时提供了特定值
            if delay is None:
                delay = self.step_delay.get().strip() or "0"
            if io_pin is None:
                io_pin = self.step_io_pin.get().strip() or "0"
            if io_state is None:
                io_state = self.step_io_state.get().strip() or "0"
            
            # 检查是否选中了某个步骤（不弹窗提示）
            selected_idx = self._selected_step_index(show_hint=False)
            
            if selected_idx is not None:
                # 检查选中的是否是空指令（type=0 或 desc 包含"空步骤"）
                selected_step = None
                for s in self.step_records:
                    if s["index"] == selected_idx:
                        selected_step = s
                        break
                
                if selected_step and (selected_step["type"] == 0 or "空步骤" in selected_step["desc"]):
                    # 替换空指令模式
                    insert_flag = 0  # 覆盖
                    idx = selected_idx  # 使用当前选中的索引
                    self._append_log(f"替换空步骤 {idx}")
                    
                    # 构建新指令
                    cmd = f"SetStep_{execute_flag},{insert_flag},{idx},{type_code},{p1},{p2},{p3},{speed},{delay},{io_pin},{io_state},"
                    
                    # 更新本地镜像
                    new_step = {"index": idx, "type": type_code, "desc": desc, "cmd": cmd}
                    self.step_records[selected_idx - 1] = new_step
                    
                    # 发送指令到下位机（需要先删除再插入，或者直接覆盖）
                    # 这里采用先删除后插入的方式
                    del_cmd = f"Delete_{idx},"
                    self._io_q.put(("send", del_cmd))
                    time.sleep(0.05)  # 短暂延时确保下位机处理完成
                    self._io_q.put(("send", cmd))
                    
                    self._refresh_step_tree()
                    self._append_log(f"发送：{cmd}  (替换步骤 {idx})")
                    return
                else:
                    # 插入模式：在选中步骤之后插入
                    insert_flag = 1  # 启用插入功能
                    idx = selected_idx + 1  # 在选中位置之后插入
                    
                    # 构建指令
                    cmd = f"SetStep_{execute_flag},{insert_flag},{idx},{type_code},{p1},{p2},{p3},{speed},{delay},{io_pin},{io_state},"
                    
                    # 更新本地镜像：在选中位置之后插入
                    new_step = {"index": idx, "type": type_code, "desc": desc, "cmd": cmd}
                    self.step_records.insert(selected_idx, new_step)
                    
                    # 重新编号所有步骤
                    for i, s in enumerate(self.step_records, start=1):
                        s["index"] = i
                    
                    self._refresh_step_tree()
                    self._append_log(f"发送：{cmd}  (在步骤 {selected_idx} 后插入)")
                    self._io_q.put(("send", cmd))
                    return
            
            # 默认模式：追加到末尾
            idx = len(self.step_records) + 1
            insert_flag = 0  # 顺序追加
            
            # SetStep 指令格式：SetStep_执行判断，插入功能，步骤数，指令类型，P1,P2,P3，速度，延时，IO 引脚，IO 状态，
            cmd = f"SetStep_{execute_flag},{insert_flag},{idx},{type_code},{p1},{p2},{p3},{speed},{delay},{io_pin},{io_state},"
        except Exception as e:
            self._append_log(f"构建步骤指令失败：{e}")
            return

        self.step_records.append({"index": idx, "type": type_code, "desc": desc, "cmd": cmd})
        self._refresh_step_tree()
        self._append_log(f"发送：{cmd}  (步骤 {idx})")
        self._io_q.put(("send", cmd))

    def _refresh_step_tree(self):
        for item in self.step_tree.get_children():
            self.step_tree.delete(item)
        for step in self.step_records:
            self.step_tree.insert("", "end", iid=str(step["index"]), values=(step["index"], step["desc"]))

    def _selected_step_index(self, show_hint: bool = True) -> int | None:
        """获取选中的步骤索引
        
        Args:
            show_hint: 是否显示提示弹窗（未选中时）
        """
        sel = self.step_tree.selection()
        if not sel:
            if show_hint:
                messagebox.showinfo("提示", "请先在步骤列表中选中一条记录。")
            return None
        try:
            return int(sel[0])
        except ValueError:
            return None

    def _delete_step(self):
        if not self._require_connected():
            return
        idx = self._selected_step_index()
        if idx is None:
            return
        # 发送下位机删除指令
        cmd = f"Delete_{idx},"
        self._append_log(f"发送：{cmd}  (删除步骤 {idx})")
        self._io_q.put(("send", cmd))
        # 本地镜像删除并重排索引
        self.step_records = [s for s in self.step_records if s["index"] != idx]
        for i, s in enumerate(self.step_records, start=1):
            s["index"] = i
        self._refresh_step_tree()

    def _insert_empty_step(self):
        """在选中的步骤之前插入一个空步骤（可用于在第一项前插入）"""
        if not self._require_connected():
            return
        
        # 不传 show_hint 参数，默认未选中时不弹窗
        idx = self._selected_step_index(show_hint=False)
        
        if idx is None:
            # 未选中任何项，在末尾插入空步骤
            idx = len(self.step_records) + 1
            insert_flag = 0  # 追加
        else:
            # 选中了某项，在其之前插入
            insert_flag = 1  # 启用插入功能
        
        # 发送下位机插入指令
        cmd = f"Insert_{idx},"
        self._append_log(f"发送：{cmd}  (在步骤 {idx} 前插入空步骤)")
        self._io_q.put(("send", cmd))
        
        # 本地镜像：插入一条占位记录
        placeholder = {"index": idx, "type": 0, "desc": "【空步骤，待覆盖】", "cmd": ""}
        self.step_records.insert(idx - 1, placeholder)
        
        # 重新编号所有步骤
        for i, s in enumerate(self.step_records, start=1):
            s["index"] = i
        
        self._refresh_step_tree()
        self._append_log(f"已插入空步骤 {idx}")

    def _clear_local_steps(self):
        # 仅清空 GUI 显示，不向下位机发送清除指令
        self.step_records = []
        self._refresh_step_tree()

    # ===== 程序配方相关指令 =====
    def _get_recipe(self, var: tk.StringVar) -> str:
        v = var.get().strip()
        if v not in {"1", "2", "3"}:
            v = "1"
        return v

    def _do_set_step_ok(self):
        if not self._require_connected():
            return
        recipe = self._get_recipe(self.recipe_var)
        cmd = f"SetStepOK_{recipe},"
        self._append_log(f"发送：{cmd}  (写入步骤到配方 {recipe})")
        self._io_q.put(("send", cmd))

    def _do_clean_recipe(self):
        if not self._require_connected():
            return
        recipe = self._get_recipe(self.recipe_var)
        cmd = f"Clean_{recipe},"
        self._append_log(f"发送：{cmd}  (清除配方 {recipe})")
        self._io_q.put(("send", cmd))

    def _do_edit_recipe(self):
        if not self._require_connected():
            return
        recipe = self._get_recipe(self.recipe_var)
        cmd = f"Editor_{recipe},"
        self._append_log(f"发送：{cmd}  (编辑配方 {recipe})")
        self._io_q.put(("send", cmd))

    def _do_single_run(self):
        if not self._require_connected():
            return
        recipe = self._get_recipe(self.run_recipe_var)
        cmd = f"Single_{recipe},"
        self._append_log(f"发送：{cmd}  (单循环运行配方 {recipe})")
        self._io_q.put(("send", cmd))

    def _do_cycle_run(self):
        if not self._require_connected():
            return
        recipe = self._get_recipe(self.run_recipe_var)
        cmd = f"Cycle_{recipe},"
        self._append_log(f"发送：{cmd}  (循环运行配方 {recipe})")
        self._io_q.put(("send", cmd))

    def _do_refer(self):
        if not self._require_connected():
            return
        cmd = "Refer"
        self._append_log("发送：Refer  (查询当前配方程序)")
        self._io_q.put(("send", cmd))

    def _do_align(self):
        if not self._require_connected():
            return
        v = self.align_entry.get().strip()
        if v == "":
            v = "0"
        cmd = f"Align_{v},"
        self._append_log(f"发送：{cmd}  (设置自动校准次数)")
        self._io_q.put(("send", cmd))

    def _do_trigger(self):
        if not self._require_connected():
            return
        cmd = "Trigger"
        self._append_log("发送：Trigger  (DI 触发配方循环)")
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