# ==============  MicroPython-ESP32 接收端  ==============
#  功能 ：ESP-NOW 接收 → 四电机 PWM 控制
#  模式0：连续 len+ang   模式1：离散 8 方向
# ========================================================

from machine import Pin, PWM      # 引脚 + PWM 外设
import espnow                     # ESP-NOW 无线接收
import network                    # 必需：启动 Wi-Fi STA 模式
import struct                     # 字节流解包（小端 16 位）
import math                       # sin/radians/π 等
import time                       # 如有需要延时（本例未用）

# ------------------------------------------------------------------
# 1. 电机方向引脚定义  IN1/IN2 为一组 H 桥，决定正反转；PWM 引脚负责调速
# ------------------------------------------------------------------
IN1_FL, IN2_FL = Pin(15, Pin.OUT), Pin(2, Pin.OUT)   # 前左
IN3_FR, IN4_FR = Pin(4, Pin.OUT),  Pin(16, Pin.OUT)  # 前右
IN3_BL, IN4_BL = Pin(13, Pin.OUT), Pin(12, Pin.OUT)  # 后左
IN1_BR, IN2_BR = Pin(14, Pin.OUT), Pin(27, Pin.OUT)  # 后右（简称 BR）

# PWM 对象：频率 20 kHz（人耳听不到），初始占空 0
PWM_FL = PWM(Pin(19), freq=20000, duty=0)  # 0-1023
PWM_FR = PWM(Pin(21), freq=20000, duty=0)
PWM_BL = PWM(Pin(25), freq=20000, duty=0)
PWM_BR = PWM(Pin(26), freq=20000, duty=0)

# ------------------------------------------------------------------
# 2. 运行参数
# ------------------------------------------------------------------
PWM_MAX = 1023          # 10 位精度上限
SMOOTH  = 0.20          # 一阶低通滤波系数（越大越“肉”）
MIN_START_PWM = 50      # 电机最低启动占空，低于此值强制 0

# 平滑滤波状态变量
smooth_FL = 0.0
smooth_FR = 0.0
smooth_BL = 0.0
smooth_BR = 0.0

# =================================================================
# 3. 电机底层封装
# =================================================================
def set_motor_dir(in_a, in_b, speed):
    """
    根据 speed 正负设置 H 桥方向
    speed 只决定方向，大小由 PWM 引脚单独给
    """
    if speed >= 0:
        in_a.value(1); in_b.value(0)
    else:
        in_a.value(0); in_b.value(1)

def write_pwm(pwm_obj, duty):
    """
    写 PWM 占空并限幅 0-1023，负值自动取 abs
    """
    duty = int(min(abs(duty), PWM_MAX))
    pwm_obj.duty(duty)

def stop_all():
    """
    紧急停止：四路 PWM 占空清零
    """
    PWM_FL.duty(0); PWM_FR.duty(0)
    PWM_BL.duty(0); PWM_BR.duty(0)

# =================================================================
# 4. 连续模式：len + ang → 差速控制
#    与 Arduino 版公式 1:1 对应
# =================================================================
def drive_continuous_LA(L, A):
    # 1. 死区：摇杆幅度太小直接停车
    if abs(L) < 0.05:
        stop_all()
        return

    # 2. 速度曲线：立方 + 线性，让低速更柔和
    speed = PWM_MAX * (0.6 * L**3 + 0.4 * L)

    # 3. 转向系数：用 sin(角度) 得到 [-1,1]
    turn_factor = math.sin(math.radians(A))

    # 4. 差速计算
    base_speed = abs(speed)
    diff = turn_factor * base_speed * 0.6   # 灵敏度可改 0.6

    if speed > 0:                           # 前进差速
        left_speed  = base_speed + diff
        right_speed = base_speed - diff
    else:                                   # 后退差速（符号反转）
        left_speed  = -base_speed - diff
        right_speed = -base_speed + diff

    # 5. 限幅
    left_speed  = max(-PWM_MAX, min(PWM_MAX, left_speed))
    right_speed = max(-PWM_MAX, min(PWM_MAX, right_speed))

    # 6. 一阶低通滤波（平滑加减速）
    global smooth_FL, smooth_FR, smooth_BL, smooth_BR
    smooth_FL = smooth_FL*(1-SMOOTH) + left_speed*SMOOTH
    smooth_FR = smooth_FR*(1-SMOOTH) + right_speed*SMOOTH
    smooth_BL = smooth_BL*(1-SMOOTH) + left_speed*SMOOTH
    smooth_BR = smooth_BR*(1-SMOOTH) + right_speed*SMOOTH

    # 7. 浮点 → 整数
    fl = int(smooth_FL)
    fr = int(smooth_FR)
    bl = int(smooth_BL)
    br = int(smooth_BR)

    # 8. 方向引脚
    set_motor_dir(IN1_FL, IN2_FL, fl)
    set_motor_dir(IN3_FR, IN4_FR, fr)
    set_motor_dir(IN3_BL, IN4_BL, bl)
    set_motor_dir(IN1_BR, IN2_BR, br)

    # 9. 最低启动占空
    pwm_fl = 0 if fl == 0 else max(abs(fl), MIN_START_PWM)
    pwm_fr = 0 if fr == 0 else max(abs(fr), MIN_START_PWM)
    pwm_bl = 0 if bl == 0 else max(abs(bl), MIN_START_PWM)
    pwm_br = 0 if br == 0 else max(abs(br), MIN_START_PWM)

    # 10. 写 PWM
    write_pwm(PWM_FL, pwm_fl)
    write_pwm(PWM_FR, pwm_fr)
    write_pwm(PWM_BL, pwm_bl)
    write_pwm(PWM_BR, pwm_br)

# =================================================================
# 5. 离散模式：8 方向固定速度
#    move_num 含义与发送端对应
# =================================================================
def control_motor(move_num):
    """
    先全部停车，再按编号给方向，PWM 固定 PWM_MAX
    如需调速可把 PWM_MAX 换成变量
    """
    stop_all()               # 先清零，避免切换抖动
    pwm = PWM_MAX            # 固定全速，可改

    # --- 四路方向真值表---
    if move_num == 0:    # 前
        IN1_FL.value(0); IN2_FL.value(1)
        IN3_FR.value(0); IN4_FR.value(1)
        IN3_BL.value(0); IN4_BL.value(1)
        IN1_BR.value(0); IN2_BR.value(1)
    elif move_num == 1:  # 后
        IN1_FL.value(1); IN2_FL.value(0)
        IN3_FR.value(1); IN4_FR.value(0)
        IN3_BL.value(1); IN4_BL.value(0)
        IN1_BR.value(1); IN2_BR.value(0)
    elif move_num == 2:  # 左平移
        IN1_FL.value(1); IN2_FL.value(0)
        IN3_FR.value(0); IN4_FR.value(1)
        IN3_BL.value(0); IN4_BL.value(1)
        IN1_BR.value(1); IN2_BR.value(0)
    elif move_num == 3:  # 右平移
        IN1_FL.value(0); IN2_FL.value(1)
        IN3_FR.value(1); IN4_FR.value(0)
        IN3_BL.value(1); IN4_BL.value(0)
        IN1_BR.value(0); IN2_BR.value(1)
    elif move_num == 4:  # 左上
        IN1_FL.value(0); IN2_FL.value(0)
        IN3_FR.value(0); IN4_FR.value(1)
        IN3_BL.value(0); IN4_BL.value(1)
        IN1_BR.value(0); IN2_BR.value(0)
    elif move_num == 5:  # 右上
        IN1_FL.value(0); IN2_FL.value(1)
        IN3_FR.value(0); IN4_FR.value(0)
        IN3_BL.value(0); IN4_BL.value(0)
        IN1_BR.value(0); IN2_BR.value(1)
    elif move_num == 6:  # 左下
        IN1_FL.value(1); IN2_FL.value(0)
        IN3_FR.value(0); IN4_FR.value(0)
        IN3_BL.value(0); IN4_BL.value(0)
        IN1_BR.value(1); IN2_BR.value(0)
    elif move_num == 7:  # 右下
        IN1_FL.value(0); IN2_FL.value(0)
        IN3_FR.value(1); IN4_FR.value(0)
        IN3_BL.value(1); IN4_BL.value(0)
        IN1_BR.value(0); IN2_BR.value(0)
    else:                # 停止或编号非法
        pwm = 0          # 占空给 0

    # 四路 PWM 写同一个占空
    PWM_FL.duty(pwm); PWM_FR.duty(pwm)
    PWM_BL.duty(pwm); PWM_BR.duty(pwm)

# =================================================================
# 6. ESP-NOW 接收回调（异步，中断级）
#    收到一包立刻解析 → 调用对应函数
# =================================================================
def recv_cb(e):
    """
    异步回调：只要有数据就进一次
    e.any() 为非零表示队列里还有包
    """
    while e.any():                      # 一次性把队列取空
        mac, msg = e.recv(0)            # mac 是发送方地址，msg 是字节数组
        if len(msg) == 8:               # 连续模式：8 字节
            # 小端 16 位解包
            L_int = struct.unpack('<h', msg[0:2])[0]
            A_int = struct.unpack('<h', msg[2:4])[0]
            # 方向反转：与发送端约定一致
            L = -L_int / 100.0
            A = -A_int / 100.0
            # 限幅保护
            A = max(-180, min(180, A))
            L = max(-1, min(1, L))
            # 交给连续模式函数
            drive_continuous_LA(L, A)

        elif len(msg) == 2 and msg[0] == 1:  # 离散模式：首字节=1
            control_motor(msg[1])           # 第二字节就是方向编号

# =================================================================
# 7. 启动流程
# =================================================================
sta = network.WLAN(network.STA_IF)   # 创建站点接口
sta.active(True)                     # 必须激活，否则 esp_now 报错
e = espnow.ESPNow()                  # 生成 ESP-NOW 对象
e.active(True)                       # 打开无线
e.irq(recv_cb)                       # 注册异步接收回调
# 注意：不需要 add_peer，接收是“ promiscuous”——
#       只要信道相同、MAC 正确就能收

print("接收端启动完成")               # 提示信息，只打印一次
