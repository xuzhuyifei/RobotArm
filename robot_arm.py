import serial

class RobotArm:

    def __init__(self):
        self.ser = None

    def connect(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=1)

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + "\n").encode())
            print("发送:", cmd)

    # =========================
    # 基本控制
    # =========================

    def speed(self, v):
        self.send(f"Speed_{v},")

    def stop(self):
        self.send("Stop")

    def origin(self):
        self.send("Origin")

    # =========================
    # 关节控制
    # =========================

    def joint(self, a1, a2, a3, speed):
        self.send(f"JointAngle_{a1},{a2},{a3},{speed},")

    def joint_offset(self, a1, a2, a3, speed):
        self.send(f"JointAngleOffset_{a1},{a2},{a3},{speed},")

    # =========================
    # 笛卡尔坐标
    # =========================

    def world(self, x, y, z, speed):
        self.send(f"DescartesPoint_{x},{y},{z},{speed},")

    def world_offset(self, x, y, z, speed):
        self.send(f"DescartesPointOffset_{x},{y},{z},{speed},")

    # =========================
    # 直线运动
    # =========================

    def line(self, x, y, z, speed):
        self.send(f"DescartesLine_{x},{y},{z},{speed},")

    def line_offset(self, x, y, z, speed):
        self.send(f"DescartesLinearOffset_{x},{y},{z},{speed},")

    # =========================
    # IO控制
    # =========================

    def di(self, pin, state):
        self.send(f"DI_{pin},{state},")

    def do(self, pin, state):
        self.send(f"DO_{pin},{state},")

    # =========================
    # 末端执行器
    # =========================

    def suction(self, state):
        self.send(f"Suction_{state},")

    def grasp(self, state):
        self.send(f"Grasp_{state},")

    # =========================
    # 第四轴
    # =========================

    def axis4(self, angle):
        self.send(f"Axis_{angle},")

    # =========================
    # 程序循环
    # =========================

    def single(self, recipe):
        self.send(f"Single_{recipe},")

    def cycle(self, recipe):
        self.send(f"Cycle_{recipe},")

    # =========================
    # 配方
    # =========================

    def clean(self, recipe):
        self.send(f"Clean_{recipe},")

    def editor(self, recipe):
        self.send(f"Editor_{recipe},")

    def set_step_ok(self, recipe):
        self.send(f"SetStepOK_{recipe},")

    # =========================
    # 步骤写入
    # =========================

    def set_step(self, exec_flag, insert_flag, step, cmd,
                 p1, p2, p3, speed, delay, io_pin, io_state):

        s = f"SetStep_{exec_flag},{insert_flag},{step},{cmd},{p1},{p2},{p3},{speed},{delay},{io_pin},{io_state},"
        self.send(s)

    # =========================
    # 查询
    # =========================

    def refer(self):
        self.send("Refer")

    def info(self):
        self.send("Infor")

    # =========================
    # 其他
    # =========================

    def align(self, n):
        self.send(f"Align_{n},")

    def rod_len(self, l):
        self.send(f"RodLen_{l},")

    def delete(self, step):
        self.send(f"Delete_{step},")

    def insert(self, step):
        self.send(f"Insert_{step},")