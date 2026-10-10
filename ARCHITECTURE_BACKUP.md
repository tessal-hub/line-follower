# BẢN LƯU TRỮ KIẾN TRÚC HỆ THỐNG (ARCHITECTURE BACKUP)
**Dự án:** ESP32 Hyper-Speed Line Follower  
**Thời điểm lưu trữ:** 09/10/2026  
**Mục đích:** Lưu trữ toàn bộ mô hình động học, cấu trúc phân tầng (cascade), bộ điều khiển độ cong (curvature), bộ điều khiển vòng kín tốc độ (speed controller) và máy trạng thái (state machine) trước khi rút gọn về điều khiển PD nguyên thủy.

---

## 1. TỔNG QUAN PHẦN CỨNG & PINOUT

### 1.1. Sơ đồ đấu nối GPIO (ESP32 NodeMCU-32S)
| GPIO | Tín hiệu / Macro | Thiết bị ngoại vi | Chế độ | Ghi chú |
|---|---|---|---|---|
| **18** | `PIN_MOTOR_LEFT_IN1` | DRV8871 Motor Trái (IN1) | PWM (LEDC Ch0) | 20 kHz, 8-bit resolution |
| **19** | `PIN_MOTOR_LEFT_IN2` | DRV8871 Motor Trái (IN2) | PWM (LEDC Ch1) | 20 kHz, 8-bit resolution |
| **26** | `PIN_MOTOR_RIGHT_IN1` | DRV8871 Motor Phải (IN1) | PWM (LEDC Ch2) | 20 kHz, 8-bit resolution |
| **25** | `PIN_MOTOR_RIGHT_IN2` | DRV8871 Motor Phải (IN2) | PWM (LEDC Ch3) | 20 kHz, 8-bit resolution |
| **32** | `PIN_ENC_LEFT_A` | Encoder Trái Kênh A | `INPUT_PULLUP` | Ngắt cạnh `CHANGE` (4x Quadrature) |
| **33** | `PIN_ENC_LEFT_B` | Encoder Trái Kênh B | `INPUT_PULLUP` | Đọc trực tiếp thanh ghi GPIO |
| **34** | `PIN_ENC_RIGHT_A` | Encoder Phải Kênh A | `INPUT` | Kéo trở ngoài 4.7k-10k lên 3.3V |
| **35** | `PIN_ENC_RIGHT_B` | Encoder Phải Kênh B | `INPUT` | Kéo trở ngoài 4.7k-10k lên 3.3V |
| **36** | `PIN_MUX_SIG` | MUX CD74HC4051 COM | Analog ADC | `ADC1_CH0` (SENSOR_VP) |
| **16** | `PIN_MUX_S0` | MUX Địa chỉ Bit 0 | Output | Ghi thanh ghi nhanh ~15ns |
| **17** | `PIN_MUX_S1` | MUX Địa chỉ Bit 1 | Output | Ghi thanh ghi nhanh ~15ns |
| **23** | `PIN_MUX_S2` | MUX Địa chỉ Bit 2 | Output | Ghi thanh ghi nhanh ~15ns |
| **21** | `PIN_I2C_SDA` | I2C OLED 1.3" SH1106 | I2C Data | 400 kHz Fast I2C (Core 0) |
| **22** | `PIN_I2C_SCL` | I2C OLED 1.3" SH1106 | I2C Clock | 400 kHz Fast I2C (Core 0) |
| **13** | `PIN_BTN_SW1` | Nút nhấn SW1 | `INPUT_PULLUP` | Idle: Calibrate 10s / Run: E-Stop |
| **14** | `PIN_BTN_SW2` | Nút nhấn SW2 | `INPUT_PULLUP` | Nhấp: RUN/STOP; Giữ >=450ms: LINE-HOLD |
| **27** | `PIN_BTN_SW3` | Nút nhấn SW3 | `INPUT_PULLUP` | Nhấp ở Idle: Mở Menu OLED NVS |

### 1.2. Thông số cơ khí & động cơ
- Động cơ: JGB37-520 12V 1590RPM
- Tỉ số truyền hộp số: $6.25:1$
- Xung Hall rotor: 11 PPR $\rightarrow$ Giải mã $4\times$ quadrature: $275.0$ ticks/vòng bánh xe.
- Đường kính bánh xe: $D = 68.0\text{ mm}$.
- Quãng đường trên mỗi xung tick: $0.7768\text{ mm/tick}$.
- Chiều rộng vết bánh xe (Track width): $W = 160.0\text{ mm}$.
- Thanh cảm biến: 8 mắt hồng ngoại trên bề rộng $83.0\text{ mm}$ (bước mắt $11.857\text{ mm}$).

---

## 2. KIẾN TRÚC PHẦN MỀM TẦNG CAO (CASCADE SYSTEM)

```
[8-IR Sensors via MUX (Core 1, 250Hz)]
               │
               ▼
[Sensors Process: Trọng số bình phương & khử sàn nhiễu]
               │ e (sai số lệch tâm, mm)
               ▼
[State Machine: RUN / SHARP_TURN / RECOVERY]
               │ Mode cờ hiệu & hệ số ép góc
               ▼
[Curvature Controller: Tính kappa_cmd -> vi sai omega_cmd]
               │ vTargetL, vTargetR (mm/s)
               ▼
[Speed Controller: Feedforward (ks, kv, ka) + PI chung + P vi sai]
               │ pwmL, pwmR (-255 .. +255)
               ▼
[Dual DRV8871 H-Bridge Motor Drivers (20 kHz PWM Fast Decay)]

[Encoders 4x ISR] ───────────────▲ (Hồi tiếp tốc độ mm/s)
```

---

## 3. CÁC MODULE ĐÃ XÂY DỰNG

### 3.1. Bộ điều khiển độ cong (`curvature.cpp`, `curvature.h`)
- Chuyển đổi sai số ngang $e\text{ (m)}$ sang độ cong quỹ đạo yêu cầu $\kappa\text{ (m}^{-1}\text{)}$:
  $$\kappa = \left(K_p \cdot e + K_d \cdot \frac{de}{dt}\right) \cdot \left(\frac{v_{ref}}{v}\right)^2$$
- Chuyển sang vận tốc góc:
  $$\omega = v \cdot \kappa$$
- Phân bổ vận tốc vi sai hai bánh theo động học vi sai (Differential Drive Kinematics):
  $$v_L = v + \frac{\omega \cdot W}{2}, \quad v_R = v - \frac{\omega \cdot W}{2}$$
- Cơ chế scale đều cả hai bánh khi một bánh vượt trần $v_{max}$ để bảo toàn góc bẻ lái $\kappa$.

### 3.2. Bộ điều khiển tốc độ đóng kín (`speed_controller.cpp`, `speed_controller.h`)
- Mô hình Feedforward động cơ JGB37-520:
  $$\text{PWM}_{FF} = k_s \cdot \text{sgn}(v) + k_v \cdot v + k_a \cdot a$$
  với $k_s \approx 15\text{ PWM}$ (ma sát tĩnh), $k_v \approx 0.115\text{ PWM/(mm/s)}$, $k_a \approx 0.002$.
- Bù sụt áp ắc quy/pin LiPo qua $V_{bat}$.
- Vòng lặp PI điều khiển vận tốc tịnh tiến trung bình $u_{PI} = K_p (v_{tgt} - v_{meas}) + \int K_i e_v dt$.
- Vòng lặp P vi sai chống trượt lốp: $u_{diff} = K_{p,diff} \cdot ((v_{tL} - v_{tR}) - (v_{mL} - v_{mR}))$.
- Tổng hợp đầu ra:
  $$\text{PWM}_L = \text{FF}_L + u_{PI} + u_{diff}$$
  $$\text{PWM}_R = \text{FF}_R + u_{PI} - u_{diff}$$

### 3.3. Máy trạng thái quỹ đạo (`state_machine.cpp`, `state_machine.h`)
- 3 trạng thái:
  1. `STATE_RUN`: Bám line thẳng và cua lượn thông thường.
  2. `STATE_SHARP_TURN`: Khi sai số $>12\text{ mm}$ hoặc vạch tràn biên ngoại suy, kích hoạt tăng quyền lái gấp $1.4\times - 2.6\times$ và cho phép bánh trong đảo chiều lùi.
  3. `STATE_RECOVERY`: Mất line liên tục $>32\text{ ms}$, kích hoạt thuật toán rà quét theo hướng nhớ cuối cùng. Sau $700\text{ ms}$ tự động phanh dừng an toàn (`emergencyStopTriggered`).

### 3.4. Module Characterization Motor (`main.cpp`)
- Tự động chạy quét 5 điểm PWM: 60, 100, 140, 180, 220.
- Đo vận tốc thực tế qua encoder và thực hiện hồi quy tuyến tính bình phương tối thiểu (Least Squares Regression) để tìm $k_s, k_v, R^2$.

### 3.5. Hệ thống hiển thị đa nhiệm Core 0 (`display.cpp`, `display.h`)
- Chạy task độc lập `oledCore0Task` trên Core 0 (ESP32 có 2 core), tách hoàn toàn khỏi vòng lặp điều khiển thời gian thực 250Hz trên Core 1.
- Giao tiếp dữ liệu snapshot qua `portMUX_TYPE dispMux` (FreeRTOS critical section).
- Hỗ trợ menu 11 mục lưu trữ Flash NVS (`Preferences.h`):
  - Boost Speed, Base Speed, Corner Speed
  - Kp Gain, Kd Gain, K-Curve Gain
  - Active Brake (ABS)
  - Save & Exit, Cancel, Restore Defaults, Test Line-Hold.

---

## 4. LÝ DO HOÀN NGUYÊN VỀ PD THUẦN TÚY (PRIMITIVE PD)
1. **Đơn giản tối đa (KISS / YAGNI):** Việc thêm các tầng Curvature + Odometry + Speed Controller + State Machine tạo ra quá nhiều tham số phụ thuộc chéo ($k_s, k_v, k_a, K_{p,curv}, K_{d,curv}, K_{p,spd}, K_{i,spd}$). Khi xe chạy thực tế trên đường đua có độ bám và điện áp pin biến thiên, việc tinh chỉnh trở nên phức tạp.
2. **Độ trễ phản hồi (Response Latency):** Thuật toán PD truyền thống tính trực tiếp sai số quang ra xung băm PWM động cơ $\rightarrow$ đáp ứng mô-men tức thì (zero filter lag), giúp xe bám line cực gắt ở tốc độ cao mà không bị trễ pha do vòng tích phân tốc độ.
3. **Dễ hiểu, dễ bảo trì:** Mọi sinh viên, kỹ sư đều có thể làm chủ thuật toán PD cơ bản chỉ với 2 nút tinh chỉnh $K_p$ và $K_d$.

Bản backup này giữ nguyên đầy đủ để tái sử dụng bất cứ khi nào cần nghiên cứu sâu hơn về điều khiển xe tự hành nâng cao.
