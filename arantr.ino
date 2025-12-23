#include <WiFi.h>
#include <esp_now.h>

// ---------- 引脚 ----------
const int ps2_y_pin = 33;   // Y 轴（横向）
const int ps2_x_pin = 32;   // X 轴（纵向）
const int btn_pin   = 23;

// ---------- ESP-NOW ----------
esp_now_peer_info_t peerInfo;

// ---------- 全局变量 ----------
int mode = 0;        // 0=连续  1=离散
int last_btn = HIGH;

long centerA = 2048;   // 实际校准中心（Y）
long centerB = 2048;   // 实际校准中心（X）
int deadzone_A = 20;   // Y 真实中立区
int deadzone_B = 20;   // X 真实中立区

// 让“摇杆向上推”成为 0° 正方向：
const float ANGLE_OFFSET = PI / 2.0f;

// =========================================================
// 自动校准摇杆中心（开机执行一次）
// =========================================================
void calibrateCenters() {
    Serial.println("=== 校准：保持摇杆中立 2 秒 ===");

    const int N = 200;
    long sumA = 0, sumB = 0;
    int samples = 0;

    int minA = 5000, maxA = -1;
    int minB = 5000, maxB = -1;

    for (int i = 0; i < N; i++) {
        int ay = analogRead(ps2_y_pin);
        int ax = analogRead(ps2_x_pin);

        if (abs(ay - 2048) < 400 && abs(ax - 2048) < 400) {
            sumA += ay;
            sumB += ax;
            samples++;

            if (ay < minA) minA = ay;
            if (ay > maxA) maxA = ay;
            if (ax < minB) minB = ax;
            if (ax > maxB) maxB = ax;
        }
        delay(10);
    }

    if (samples > 20) {
        centerA = sumA / samples;
        centerB = sumB / samples;
    }

    deadzone_A = max(60, (maxA - minA) / 2 + 15);
    deadzone_B = max(60, (maxB - minB) / 2 + 15);

    Serial.println("=== 校准完成 ===");
    Serial.printf("Y中心=%ld  死区=%d\n", centerA, deadzone_A);
    Serial.printf("X中心=%ld  死区=%d\n", centerB, deadzone_B);
}

// =========================================================
void setup() {
    Serial.begin(115200);
    pinMode(ps2_y_pin, INPUT);
    pinMode(ps2_x_pin, INPUT);
    pinMode(btn_pin, INPUT_PULLUP);

    WiFi.mode(WIFI_STA);
    esp_now_init();

    uint8_t peerMac[] = {0x68, 0xFE, 0x71, 0xA5, 0x6B, 0x80};
    memcpy(peerInfo.peer_addr, peerMac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    calibrateCenters();
    Serial.println("发送端启动完成");
}

// =========================================================
void loop() {
    int rawY = analogRead(ps2_y_pin);
    int rawX = analogRead(ps2_x_pin);

    int horizontal = rawY;
    int vertical   = rawX;

    int centerH = centerA;
    int centerV = centerB;

    // ----------- 切换模式 ----------
    int btn = digitalRead(btn_pin);
    if (btn == LOW && last_btn == HIGH) {
        mode = 1 - mode;
        Serial.printf("模式切换 -> %d\n", mode);
        delay(300);
    }
    last_btn = btn;

    // ======================================================
    // 模式 0：二维向量（len+ang）
    // ======================================================
    if (mode == 0) {

        float x_raw = horizontal - centerH;   // 左右方向
        float y_raw = vertical   - centerV;   // 上下方向

        // ---------- 真实中立区 ----------
        float r = sqrtf(x_raw * x_raw + y_raw * y_raw);
        if (abs(x_raw) < deadzone_A && abs(y_raw) < deadzone_B) {
            x_raw = 0;
            y_raw = 0;
            r = 0;
        }

        // ---------- 归一化 ----------
        const float R_MAX = 1800.0f;
        float len = r / R_MAX;
        if (len > 1.0f) len = 1.0f;

        // ========== 下半平面 -1 显示（方便接收端处理后退模式）==========
        if (y_raw > 0) {          // 下推
            len = - (r / R_MAX);
            if (len < -1.0f) len = -1.0f;
        }

        // ---------- 方向角（数学向量） ----------
        float ang = atan2f(y_raw, x_raw);

        // ---------- 修正方向：摇杆上推 = 正方向（0°） ----------
        ang += ANGLE_OFFSET;

        // ---------- 角度归一化 ----------
        if(ang > PI)  ang -= 2 * PI;
        if(ang < -PI) ang += 2 * PI;

        // ---------- 打包 ----------
        uint8_t buf[8];
        int16_t L_int = (int16_t)(len * 100);      // 长度转整数 0~100
        int16_t A_int = (int16_t)(ang * 180.0f / PI * 100);  // 角度转整数 -18000~18000

        // 小端序打包到buf
        buf[0] = L_int & 0xFF;        // L低字节
        buf[1] = (L_int >> 8) & 0xFF; // L高字节
        buf[2] = A_int & 0xFF;        // A低字节
        buf[3] = (A_int >> 8) & 0xFF; // A高字节
        // 剩余4字节补0（保持8字节长度兼容）
        buf[4] = 0; buf[5] = 0; buf[6] = 0; buf[7] = 0;

        esp_now_send(peerInfo.peer_addr, buf, 8);

        // 调试输出（角度用A_int/100.0）
        Serial.printf("[Mode0] len=%.2f  ang=%.1f°\n", L_int / 100.0f, A_int / 100.0f);
        }

    // ======================================================
    // 模式 1：离散方向（保持原样）
    // ======================================================
    else {
        int dead = 300;   //前后左右区间阱
        int move_num = 8;

        bool up    = (vertical   < centerV - dead);
        bool down  = (vertical   > centerV + dead);
        bool right = (horizontal < centerH - dead);
        bool left  = (horizontal > centerH + dead);

        if (up && left)        move_num = 4;
        else if (up && right)  move_num = 5;
        else if (down && left) move_num = 6;
        else if (down && right)move_num = 7;
        else if (left)         move_num = 2;
        else if (right)        move_num = 3;
        else if (up)           move_num = 0;
        else if (down)         move_num = 1;

        uint8_t data[2] = {1, (uint8_t)move_num};
        esp_now_send(peerInfo.peer_addr, data, 2);

        Serial.printf("[Mode1] move_num=%d\n", move_num);
    }

    delay(30);
}
