# =============  MicroPython-ESP32 发送端  =============
#  功能：摇杆 ADC → ESP-NOW 无线数据 → 接收端
#  模式0：连续 len+ang   模式1：离散 8 方向
# ======================================================

from machine import Pin, ADC, Timer   # 硬件抽象：ADC 读摇杆，Pin 读按键，Timer 做周期
import espnow                         # ESP-NOW 无线广播
import network                        # 必需的 WiFi 占位（ESP-NOW 必须开 STA）
import math                           # 三角函数、atan2、平方根
import time                           # 延时 ms

# ---------- 硬件引脚定义 ----------
ps2_y_pin = ADC(Pin(33))      # 摇杆 Y 轴（上下）接到 GPIO33
ps2_x_pin = ADC(Pin(32))      # 摇杆 X 轴（左右）接到 GPIO32
btn_pin   = Pin(23, Pin.IN, Pin.PULL_UP)  # 按键→GPIO23，内部上拉（按下低电平）

# ADC 量程 0-3.3 V → 0-4095，11DB 衰减必须写，否则满量程不对
ps2_y_pin.atten(ADC.ATTN_11DB)
ps2_x_pin.atten(ADC.ATTN_11DB)

# ---------- ESP-NOW 初始化 ----------
sta = network.WLAN(network.STA_IF)   # 创建站点接口（ESP-NOW 要求）
sta.active(True)                     # 一定要激活，否则 esp_now 报错
e = espnow.ESPNow()                  # 生成 ESP-NOW 对象
e.active(True)                       # 打开无线外设
PEER_MAC = b'\x68\xFE\x71\xA5\x6B\x80'  # 接收端 MAC（与接收代码严格一致）
e.add_peer(PEER_MAC)                 # 把对端加入白名单，才能发单播

# ---------- 全局变量 ----------
mode      = 0           # 0=连续向量模式  1=离散方向模式
last_btn  = 1           # 按键去抖：记录上一次电平
centerA   = 2048        # Y 轴“中立”ADC 值（校准后更新）
centerB   = 2048        # X 轴“中立”ADC 值（校准后更新）
deadzone_A = 20         # Y 轴死区（校准后还会再算一遍）
deadzone_B = 20         # X 轴死区（同上）
ANGLE_OFFSET = math.pi / 2.0  # 把“摇杆上推”当成 0° 的相位修正（π/2）

# =================================================
#  自动校准中立点（开机只跑一遍）
#  让摇杆保持居中 2 秒 → 计算真实中心 + 死区
# =================================================
def calibrate_centers():
    print("=== 校准：保持摇杆中立 2 秒 ===")
    N = 200                 # 采样 200 次，间隔 10 ms → 约 2 秒
    sumA = 0; sumB = 0; samples = 0
    minA = 5000; maxA = -1
    minB = 5000; maxB = -1

    for _ in range(N):
        ay = ps2_y_pin.read()   # 读 ADC
        ax = ps2_x_pin.read()
        # 只采集“接近中心”的数据，剔除误触
        if abs(ay - 2048) < 400 and abs(ax - 2048) < 400:
            sumA += ay; sumB += ax; samples += 1
            minA = min(minA, ay); maxA = max(maxA, ay)
            minB = min(minB, ax); maxB = max(maxB, ax)
        time.sleep_ms(10)

    global centerA, centerB, deadzone_A, deadzone_B
    if samples > 20:                   # 有效样本足够才更新
        centerA = sumA // samples
        centerB = sumB // samples
    # 死区 = 半峰宽 + 15，且最小 50（防抖动）
    deadzone_A = max(50, (maxA - minA) // 2 + 15)
    deadzone_B = max(50, (maxB - minB) // 2 + 15)
    print("=== 校准完成 ===")
    print("Y中心=%d  死区=%d" % (centerA, deadzone_A))
    print("X中心=%d  死区=%d" % (centerB, deadzone_B))

# =================================================
#  主循环 —— 每 30 ms 被硬件定时器调用一次
# =================================================
def main_loop(_):
    global mode, last_btn

    # ---- 读原始 ADC ----
    rawY = ps2_y_pin.read()
    rawX = ps2_x_pin.read()
    horizontal = rawY          
    vertical   = rawX
    centerH = centerA
    centerV = centerB

    # ---- 按键模式切换（带简单去抖） ----
    btn = btn_pin.value()
    if btn == 0 and last_btn == 1:   # 检测到“按下”沿
        mode = 1 - mode              # 0↔1 切换
        time.sleep_ms(300)           # 简单阻塞去抖
    last_btn = btn

    # =================================================
    #  模式 0 ：连续向量  len∈[-1,1]   ang∈[-180,180]
    # =================================================
    if mode == 0:
        # 1. 去中心
        x_raw = float(horizontal - centerH)
        y_raw = float(vertical   - centerV)

        # 2. 死区 → 强制归零
        r = math.sqrt(x_raw*x_raw + y_raw*y_raw)
        if abs(x_raw) < deadzone_A and abs(y_raw) < deadzone_B:
            x_raw = 0.0; y_raw = 0.0; r = 0.0

        # 3. 长度归一化 + 下半平面负值（倒车）
        R_MAX = 1800.0
        len = r / R_MAX
        if len > 1.0: len = 1.0
        if y_raw > 0:              # 摇杆向下 → 负数（后退）
            len = -(r / R_MAX)
            if len < -1.0: len = -1.0

        # 4. 角度计算 → 上推为 0°
        ang = math.atan2(y_raw, x_raw)          # 向量角
        ang += ANGLE_OFFSET                     # 相位旋转 90°
        # 归一化到 [-π, π]
        if ang >  math.pi: ang -= 2 * math.pi
        if ang < -math.pi: ang += 2 * math.pi

        # 5. 打包成小端 16 位整数
        L_int = int(len * 100)                # -100~100
        A_int = int(ang * 180.0 / math.pi * 100)  # -18000~18000

        buf = bytearray(8)                    # 固定 8 字节（兼容协议）
        buf[0] = L_int & 0xFF
        buf[1] = (L_int >> 8) & 0xFF
        buf[2] = A_int & 0xFF
        buf[3] = (A_int >> 8) & 0xFF
        # 剩余 4 字节保持 0（接收端原样解析）

        # 6. 发送（带异常消化，超时/队列满都不打印）
        try:
            e.send(PEER_MAC, buf)
        except OSError:
            pass                           # 静默丢弃，不刷屏

    # =================================================
    #  模式 1 ：离散 8 方向
    # =================================================
    else:
        # 矩形 → 菱形（L1 范数）
        straight_thr = 300          # 单轴直线阈值
        diag_thr     = 250          # 菱形斜向阈值（可再降到 200-220）

        dx = horizontal - centerH
        dy = vertical   - centerV

        # 先判斜向（菱形）
        if abs(dx) + abs(dy) > diag_thr:
        if dx > 0 and dy > 0:   move_num = 6   # 左下
        if dx < 0 and dy > 0:   move_num = 7   # 右下
        if dx > 0 and dy < 0:   move_num = 4   # 左上
        if dx < 0 and dy < 0:   move_num = 5   # 右上
        # 再判直线（单轴）
        elif abs(dx) > straight_thr:
        move_num = 2 if dx > 0 else 3          # 左/右
        elif abs(dy) > straight_thr:
        move_num = 0 if dy < 0 else 1          # 上/下
        else:
        move_num = 8                             # 停止

        data = bytearray([1, move_num])   # 协议：首字节=1 表示离散包
        try:
            e.send(PEER_MAC, data)
        except OSError:
            pass                           # 同样静默处理超时

# ----------------  启动  ----------------
calibrate_centers()          # 上电先校准
print("发送端启动完成")       # 提示校准结束

# 硬件定时器 30 ms 周期调用 main_loop，永不阻塞
Timer(-1).init(period=30, mode=Timer.PERIODIC, callback=main_loop)
