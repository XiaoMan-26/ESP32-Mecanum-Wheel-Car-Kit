#include <WiFi.h>
#include <esp_now.h>
#include <math.h>

// ===================== 电机引脚定义 =====================
const int IN1_FL = 15, IN2_FL = 2;
const int IN3_FR = 4,  IN4_FR = 16;
const int IN3_BL = 13, IN4_BL = 12;
const int IN1_BR2 = 14, IN2_BR2 = 27;

const int PWM_FL = 19;
const int PWM_FR = 21;
const int PWM_BL = 25;
const int PWM_BR2 = 26;

// ===================== PWM 参数配置 =====================
const uint32_t PWM_FREQ = 20000;  // 20kHz
const uint8_t  PWM_RES  = 10;     // 10位精度（0~1023）决定PWM可以输出的最大值

// ===================== 运行参数设置 =====================
float smooth_FL = 0, smooth_FR = 0;
float smooth_BL = 0, smooth_BR2 = 0;

const int PWM_MAX = 1023;     // 最大 PWM（0~1023）
const float SMOOTH = 0.20f;   // 平滑权重（意思是：新速度只取 20%，老速度保留 80%）
const int MIN_START_PWM = 50;  //电机“肯转”的最小格子数

// 设置电机方向（正转 / 反转）
void setMotorDir(int INa, int INb, int speed) {
  if (speed >= 0) {
    digitalWrite(INa, HIGH);
    digitalWrite(INb, LOW);
  } else {
    digitalWrite(INa, LOW);
    digitalWrite(INb, HIGH);
  }
}

// 停车
void stopAllMotors() {
  ledcWrite(PWM_FL, 0);
  ledcWrite(PWM_FR, 0);
  ledcWrite(PWM_BL, 0);
  ledcWrite(PWM_BR2, 0);
}

// ===================== 连续模式：角度 + 长度 =====================
// 输入：L(0~1)，A(度)
// ===================== 连续模式：前进 + 角度转向 =====================
void drive_continuous_LA(float L, float A) {
    if (fabs(L) < 0.05f) {   // 死区
        stopAllMotors();
        return;
    }

    // 速度：L 范围 [-1, 1]，映射到 [-PWM_MAX, PWM_MAX]，立方非线性 
    float speed = PWM_MAX * (0.6f * L * L * L + 0.4f * L);

    // 转向角度：A 范围 [-180, 180]
    // 我们将其映射到一个转向系数 [-1, 1]
    float turn_factor = sin(radians(A));  // -1 左转到头，+1 右转到头

    // 差速转向：左右轮速度差
    float base_speed = fabs(speed);
    float diff = turn_factor * base_speed * 0.6f;  // 可调转向灵敏度

    float leftSpeed, rightSpeed;

    if (speed > 0) {
        // ===== 前进差速 =====
        leftSpeed  =  base_speed + diff;  // 左偏 → 右快
        rightSpeed =  base_speed - diff;  // 右偏 → 左快
    } else {
        // ===== 后退差速（方向反转）=====
        leftSpeed  = -base_speed - diff;  // 左倒 → 右快（倒车）
        rightSpeed = -base_speed + diff;  // 右倒 → 左
    }

    // 限幅
    leftSpeed = constrain(leftSpeed, -PWM_MAX, PWM_MAX);
    rightSpeed = constrain(rightSpeed, -PWM_MAX, PWM_MAX);

    // 平滑滤波（“抹平抖动”）leftSpeed 是刚才公式算出的“目标速度”，会跳动；smooth_FL 是“上一轮”已经平缓下来的速度。
    smooth_FL = smooth_FL*(1-SMOOTH) + leftSpeed*SMOOTH;
    smooth_FR = smooth_FR*(1-SMOOTH) + rightSpeed*SMOOTH;
    smooth_BL = smooth_BL*(1-SMOOTH) + leftSpeed*SMOOTH;
    smooth_BR2 = smooth_BR2*(1-SMOOTH) + rightSpeed*SMOOTH;

  // 转整数（“去掉小数”）  ESP32 的 ledcWrite() 只认整数，所以把带小数的平滑值向下取整
    int fl = (int)smooth_FL;
    int fr = (int)smooth_FR;
    int bl = (int)smooth_BL;
    int br = (int)smooth_BR2;

    // 设置方向（“打方向盘”）
    setMotorDir(IN1_FL, IN2_FL, fl);
    setMotorDir(IN3_FR, IN4_FR, fr);
    setMotorDir(IN3_BL, IN4_BL, bl);
    setMotorDir(IN1_BR2, IN2_BR2, br);

    // 最小启动 PWM（“给起步费”）
    int pwmFL = (fl==0 ? 0 : max(abs(fl), MIN_START_PWM));
    int pwmFR = (fr==0 ? 0 : max(abs(fr), MIN_START_PWM));
    int pwmBL = (bl==0 ? 0 : max(abs(bl), MIN_START_PWM));
    int pwmBR = (br==0 ? 0 : max(abs(br), MIN_START_PWM));

    // 输出 PWM（“踩电门”） 把最终占空比交给 ESP32 的硬件 PWM 模块，电机就按这个速度转
    ledcWrite(PWM_FL, pwmFL);
    ledcWrite(PWM_FR, pwmFR);
    ledcWrite(PWM_BL, pwmBL);
    ledcWrite(PWM_BR2, pwmBR);
}

// ===================== 离散模式控制 =====================
void control_motor(uint8_t move_num) {
    Serial.print("Mode1 move_num: ");
    Serial.println(move_num);
    int pwm = PWM_MAX;
    switch(move_num) {
        case 0: // 前
            digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, HIGH);
            digitalWrite(IN3_FR, LOW);  digitalWrite(IN4_FR, HIGH);
            digitalWrite(IN3_BL, LOW);  digitalWrite(IN4_BL, HIGH);
            digitalWrite(IN1_BR2, LOW); digitalWrite(IN2_BR2, HIGH);
            break;
        case 1: // 后
            digitalWrite(IN1_FL, HIGH);  digitalWrite(IN2_FL, LOW);
            digitalWrite(IN3_FR, HIGH);  digitalWrite(IN4_FR, LOW);
            digitalWrite(IN3_BL, HIGH);  digitalWrite(IN4_BL, LOW);
            digitalWrite(IN1_BR2, HIGH); digitalWrite(IN2_BR2, LOW);
            break;
        case 2: // 左平移
            digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
            digitalWrite(IN3_FR, LOW);  digitalWrite(IN4_FR, HIGH);
            digitalWrite(IN3_BL, LOW);  digitalWrite(IN4_BL, HIGH);
            digitalWrite(IN1_BR2, HIGH); digitalWrite(IN2_BR2, LOW);
            break;
        case 3: // 右平移
            digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, HIGH);
            digitalWrite(IN3_FR, HIGH); digitalWrite(IN4_FR, LOW);
            digitalWrite(IN3_BL, HIGH); digitalWrite(IN4_BL, LOW);
            digitalWrite(IN1_BR2,LOW);  digitalWrite(IN2_BR2, HIGH);
            break;
        case 4: // 左上
            digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, LOW);
            digitalWrite(IN3_FR, LOW); digitalWrite(IN4_FR, HIGH);
            digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, HIGH);
            digitalWrite(IN1_BR2,LOW); digitalWrite(IN2_BR2,LOW);
            break;
        case 5: // 右上
            digitalWrite(IN1_FL, LOW); digitalWrite(IN2_FL, HIGH);
            digitalWrite(IN3_FR, LOW); digitalWrite(IN4_FR, LOW);
            digitalWrite(IN3_BL, LOW); digitalWrite(IN4_BL, LOW);
            digitalWrite(IN1_BR2,LOW); digitalWrite(IN2_BR2,HIGH);
            break;
        case 6: // 左下
            digitalWrite(IN1_FL, HIGH); digitalWrite(IN2_FL, LOW);
            digitalWrite(IN3_FR, LOW);  digitalWrite(IN4_FR, LOW);
            digitalWrite(IN3_BL, LOW);  digitalWrite(IN4_BL, LOW);
            digitalWrite(IN1_BR2,HIGH); digitalWrite(IN2_BR2,LOW);
            break;
        case 7: // 右下
            digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, LOW);
            digitalWrite(IN3_FR, HIGH); digitalWrite(IN4_FR, LOW);
            digitalWrite(IN3_BL, HIGH); digitalWrite(IN4_BL, LOW);
            digitalWrite(IN1_BR2,LOW);  digitalWrite(IN2_BR2,LOW);
            break;
        case 8: // 停止
        default:
            digitalWrite(IN1_FL, LOW);  digitalWrite(IN2_FL, LOW);
            digitalWrite(IN3_FR, LOW);  digitalWrite(IN4_FR, LOW);
            digitalWrite(IN3_BL, LOW);  digitalWrite(IN4_BL, LOW);
            digitalWrite(IN1_BR2, LOW); digitalWrite(IN2_BR2, LOW);
            pwm = 0;
            break;
    }
    // 输出 PWM（“踩电门”） 把最终占空比交给 ESP32 的硬件 PWM 模块，电机就按这个速度转
    ledcWrite(PWM_FL, pwm);
    ledcWrite(PWM_FR, pwm);
    ledcWrite(PWM_BL, pwm);
    ledcWrite(PWM_BR2, pwm);
}

/*********************************************************************
 * ESP-NOW 接收回调
 * – data：真正收到的字节数组。
 * – len：数组长度，用来区分“连续模式”还是“离散模式”。
 *前 4 字节 = 力度 L，后 4 字节 = 角度 A，每 2 字节一个 int16_t。
 *********************************************************************/
void onDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == 8) {  // 连续模式
        // 【核心】16位整数解析
        int16_t L_int = (int16_t)(data[0] | (data[1] << 8));
        int16_t A_int = (int16_t)(data[2] | (data[3] << 8));
        
        // 转回浮点数
        float L = L_int / 100.0f;
        float A = A_int / 100.0f;

        // 方向反转修正（与摇杆现在规定的正方向一致而不是自带默认的正方向）
          L = -L;
          A = -A;
        
        // 防错处理
        if (A < -180) A = -180; if (A > 180) A = 180;
        if (L < -1) L = -1; if (L >  1) L =  1;   // 允许 -1 ~ 1

        // 【调试】打印解析结果
        Serial.printf("解析: L=%.2f A=%.1f°\n", L, A);

        // 运动计算
        float ang_rad = radians(A);
        float vx = L * cos(ang_rad) * PWM_MAX;
        float omega = sin(ang_rad) * L * 2.0f * (PWM_MAX / 100.0f);
        omega = constrain(omega, -250, 250);
        drive_continuous_LA(L, A);
        Serial.printf("→ vx:%.0f ω:%.0f\n", vx, omega);
    } 
    else if(len == 2 && data[0] == 1) {
        uint8_t move = data[1];
        control_motor(move);
    }
}

// ===================== Setup =====================
void setup() {
    Serial.begin(115200);

    pinMode(IN1_FL, OUTPUT); pinMode(IN2_FL, OUTPUT);
    pinMode(IN3_FR, OUTPUT); pinMode(IN4_FR, OUTPUT);
    pinMode(IN3_BL, OUTPUT); pinMode(IN4_BL, OUTPUT);
    pinMode(IN1_BR2, OUTPUT); pinMode(IN2_BR2, OUTPUT);

    // PWM 初始化
    ledcAttach(PWM_FL, PWM_FREQ, PWM_RES);
    ledcAttach(PWM_FR, PWM_FREQ, PWM_RES);
    ledcAttach(PWM_BL, PWM_FREQ, PWM_RES);
    ledcAttach(PWM_BR2, PWM_FREQ, PWM_RES);

    // WiFi & ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(onDataRecv);
    Serial.println("接收端启动完成");
}

// ===================== Loop =====================
void loop() {
    delay(20);
}
 
