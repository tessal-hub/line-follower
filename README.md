# ESP32 High-Speed Line Follower Robot

Dự án xe dò line tốc độ cao sử dụng vi điều khiển **ESP32**, mạch công suất **DRV8871 (Slow Decay PWM 16kHz)**, thanh cảm biến quang 8 kênh qua IC dồn kênh **CD74HC4051**, và vòng điều khiển **2 lớp**: lái PD + K-Curve ở tần số **2000Hz** kết hợp **vòng lặp kín vận tốc từng bánh xe (FF+PI, Encoder Quadrature 4x)** — tự bù sụt áp pin và sai khác động cơ để giữ thông số tuning ổn định từ đầu đến cuối trận.

---

## 1. Sơ đồ chân phần cứng (Hardware Pinout)

| Thành phần | Tên chân | ESP32 GPIO | Ghi chú phần cứng |
| :--- | :--- | :--- | :--- |
| **Động cơ Trái (DRV8871)** | IN1 | **GPIO 25** | LEDC Channel 0, 12-bit PWM @ 16kHz |
| | IN2 | **GPIO 26** | LEDC Channel 1, 12-bit PWM @ 16kHz |
| **Động cơ Phải (DRV8871)** | IN1 | **GPIO 19** | LEDC Channel 2, 12-bit PWM @ 16kHz |
| | IN2 | **GPIO 18** | LEDC Channel 3, 12-bit PWM @ 16kHz |
| **Encoder Trái (JGB37-520)**| Kênh A | **GPIO 32** | Ngắt phần cứng CHANGE (Pull-up nội) |
| | Kênh B | **GPIO 33** | Ngắt phần cứng CHANGE (Pull-up nội) |
| **Encoder Phải (JGB37-520)**| Kênh A | **GPIO 34** | ⚠️ **INPUT ONLY: BẮT BUỘC TRỞ KÉO NGOÀI 4.7kΩ - 10kΩ lên 3.3V** |
| | Kênh B | **GPIO 35** | ⚠️ **INPUT ONLY: BẮT BUỘC TRỞ KÉO NGOÀI 4.7kΩ - 10kΩ lên 3.3V** |
| **MUX Cảm biến (CD74HC4051)**| S0 | **GPIO 16** | Chọn kênh Bit 0 |
| | S1 | **GPIO 17** | Chọn kênh Bit 1 |
| | S2 | **GPIO 23** | Chọn kênh Bit 2 |
| | SIG | **GPIO 36 (VP)** | Ngõ vào Analog ADC1 (12-bit, suy hao 11dB) |
| **Nút bấm (Active LOW)** | SW1 | **GPIO 13** | Calib / Dừng khẩn cấp (Emergency Stop) / Cancel |
| | SW2 | **GPIO 14** | Chạy Base Speed / Giảm giá trị Menu |
| | SW3 | **GPIO 27** | Chạy Fast Speed / Giữ >600ms vào Menu / Tăng giá trị |
| **Chỉ báo trạng thái** | Buzzer | **GPIO 4** | Còi báo chủ động (Active buzzer) |
| | RGB - Đỏ | **GPIO 5** | Active LOW (Báo phanh / Lỗi) |
| | RGB - Xanh lá | **GPIO 2** | Active LOW (Báo bắt line / Sẵn sàng) |
| | RGB - Xanh dương | **GPIO 15** | Active LOW (Báo chế độ chạy / Pivot) |
| **Màn hình OLED 1.3"** | SDA | **GPIO 21** | I2C Hardware Bus (400kHz) |
| | SCL | **GPIO 22** | I2C Hardware Bus (400kHz) |

> ⚠️ **LƯU Ý ĐẶC BIỆT VỀ GPIO 34 VÀ 35:**
> ESP32 có các chân GPIO 34, 35, 36, 39 là các chân chỉ nhận tín hiệu vào (*Input Only*), silicon bên trong vi điều khiển **hoàn toàn không có điện trở kéo lên/xuống (no internal pull-up/pull-down)**. Nếu gắn Encoder Kênh A/B của bánh phải vào GPIO 34 và 35 mà không hàn trở kéo ngoài (4.7kΩ đến 10kΩ nối lên 3.3V), tín hiệu ngắt sẽ bị thả nổi (*floating*), đếm sai xung hoặc mất hoàn toàn xung Encoder bánh phải.

---

## 2. Quy trình Cân chỉnh Cảm biến (Sensor Calibration)

Để robot bám đường chính xác dưới mọi điều kiện ánh sáng và bề mặt sân:

1. **Vào chế độ Calib:** Tại màn hình `IDLE`, nhấn nút **SW1**. Còi sẽ kêu bíp ngắn và màn hình hiển thị thanh biểu đồ 8 mắt cảm biến trực tiếp.
2. **Quét cảm biến:** Cầm robot rê qua lại trên đoạn đường mẫu sao cho **tất cả 8 mắt cảm biến** đều được lướt qua cả phần **nền trắng** lẫn phần **vạch đen**.
3. **Kiểm tra độ chênh lệch:** Khi một mắt cảm biến nhận đủ dải tương phản (giá trị vạch đen trừ nền trắng $\ge 400$), phía dưới cột mắt đó sẽ xuất hiện dấu `v`. Khi đạt đủ **8/8 mắt OK**, còi sẽ kêu bíp báo hiệu.
4. **Lưu dữ liệu:**
   - Nhấn **SW1** để lưu: Toàn bộ ngưỡng phân tách đồng nhất 55% ($Threshold = Black + (White - Black) \times 0.55$) được tính toán và lưu vĩnh viễn vào bộ nhớ flash NVS (`Preferences`).
   - Nhấn **SW2** để hủy bỏ và khôi phục lại giá trị calib cũ.

---

## 3. Thứ tự Tinh chỉnh Tham số (Tuning Sequence Guide)

Để đạt tốc độ tối đa mà không bị văng line, hãy giữ **SW3 (>600ms)** để mở Menu OLED và tinh chỉnh theo đúng thứ tự 5 bước sau:

```
[1. Base Spd] ➔ [2. Kp Gain] ➔ [3. Kd Gain] ➔ [4. Accel] ➔ [5. Brk Pwr & Pos] ➔ [6. K-Curve & Bst] ➔ [7. W-Loop]
```

1. **Bước 1: Tốc độ cơ sở (`Base Spd`)**
   - Đặt `Kp = 12..15`, `Kd = 40..60`, `Brk Pwr = OFF (0%)`.
   - Bắt đầu với `Base Spd` an toàn (~110 đến 130). Nhấn SW2 cho xe chạy thử trên các đoạn cua thoải. Đảm bảo xe chạy ổn định trước khi tăng tốc.
2. **Bước 2: Hệ số tỉ lệ lái (`Kp Gain`)**
   - Tăng `Kp` từng bước (+1.0) cho đến khi xe phản ứng nhanh và ôm sát tim đường ở các đoạn cua tròn.
   - Nếu `Kp` quá cao, xe sẽ lắc lư liên tục (dao động tuần hoàn quanh line).
3. **Bước 3: Hệ số đạo hàm dập lắc (`Kd Gain`)**
   - Tăng `Kd` từng bước (+5.0) để triệt tiêu dao động lắc đuôi sinh ra bởi `Kp`.
   - `Kd` giúp xe bám thẳng tắp và êm ái khi ra khỏi góc cua.
4. **Bước 4: Gia tốc đường thẳng (`Accel`)**
   - Sau khi xe ôm cua ngọt ngào, nâng `Accel` (bước gia tốc mỗi 4ms) từ 20 lên 30..40.
   - Giúp xe nhanh chóng đạt vận tốc cực đại ngay khi nhìn thấy đoạn thẳng.
5. **Bước 5: Cường độ và Vị trí Phanh Chủ Động (`Brk Pwr` & `Brk Pos`)**
   - Bật `Brk Pwr` lên 60% .. 80%.
   - `Brk Pos` (mặc định 2600): Khi vạch lệch quá ngưỡng này (chuẩn bị vào cua gắt hoặc góc vuông 90°), thuật toán sẽ tự động kích hoạt xung hãm ngược bất đối xứng dựa trên động năng thực tế đo từ Encoder.
   - Khi vận tốc thực đã hạ về mức an toàn ôm cua (`cornerMmS`), phanh lập tức nhả để xe lướt vào cua mà không bị giật.
6. **Bước 6: Khúc cua gắt phi tuyến & Tốc độ bứt phá (`K-Curve` & `Bst Spd`)**
   - Tăng `K-Curve` (4.0 .. 8.0) để tạo thêm lực xoắn bẻ lái phi tuyến khi gặp góc vuông 90° hoặc cua tay áo.
   - Tăng `Bst Spd` (tốc độ tối đa khi chạy nút SW3) để đạt thành tích thời gian ngắn nhất trên đường thẳng dài.
7. **Bước 7: Vòng lặp kín vận tốc bánh xe (`W-Loop`)**
   - `W-Loop = ON` (mặc định): Encoder đo vận tốc thực từng bánh, bộ FF+PI tự bù PWM để bánh đạt đúng tốc độ lệnh — bù sụt áp pin khi pin yếu, bù sai khác động cơ trái/phải, giữ quãng đường nhất quán giữa các lượt chạy.
   - Nếu xe có hiện tượng cố tốc độ lạ hoặc encoder một bánh lỗi (GPIO 34/35 thiếu trở kéo ngoài): đặt `W-Loop = OFF` để quay về điều khiển PWM trực tiếp như bản cũ.
   - Vòng lặp kín **tự tạm ngưng** khi xe phanh ngược hoặc xoay pivot (những pha này ghi PWM thô), không cần can thiệp.

---

## 4. Các Cải tiến Kỹ thuật Quan trọng

1. **DRV8871 Slow Decay PWM (16kHz):**
   - Chuyển đổi từ Fast Decay (Coast ở pha ngắt xung) sang **Slow Decay** (Brake tuần hoàn dòng ở pha ngắt xung).
   - Tăng tuyến tính mô-men xoắn ở dải duty thấp, triệt tiêu vùng chết động cơ (*deadzone*).
   - Tần số 16kHz hoàn toàn nằm trong giới hạn phần cứng LEDC 12-bit của ESP32 (80MHz / 4096 = 19.53kHz) và triệt tiêu tiếng rít khó chịu.
2. **Kẹp an toàn hãm ngược (`MAX_PLUG_PWM = 3000`):**
   - Hàm `speed_run` tự động giới hạn xung âm ở `-MAX_PLUG_PWM` (~73% công suất tối đa).
   - Ngăn chặn triệt để dòng ngược đột biến từ động cơ gây sụt áp 12V và sập nguồn (Brownout Reset) cho vi điều khiển ESP32 khi vào cua gắt.
3. **Thoát phanh trực tiếp bằng Encoder vật lý:**
   - Loại bỏ phụ thuộc vào mô hình quán tính trễ ($\tau \approx 80\text{ms}$). Vòng lặp kín thoát phanh ngay khi Encoder đo được vận tốc thực đạt ngưỡng an toàn, loại bỏ hiện tượng giật cục của pseudo-ABS.
4. **Đồng bộ trạng thái Encoder chống Glitch:**
   - Bộ giải mã Quadrature 4x đọc thanh ghi `GPIO.in1.data` trực tiếp (~15ns) luôn cập nhật `lastState`, ngăn ngừa lệch pha vĩnh viễn khi gặp xung nhiễu 2-bit.
5. **Tối ưu hóa thời gian quét cảm biến (~200µs):**
   - Chuyển kênh MUX bằng **1 lần ghi thanh ghi GPIO trực tiếp** thay vì 3 lần `digitalWrite`, đọc ADC qua API ESP-IDF `adc1_get_raw` (nhanh hơn `analogRead`), thời gian ổn định MUX 2µs.
   - Giữ chu kỳ quét đủ ngắn để vòng lặp Master **2000Hz** chạy chính xác 100% (tải CPU ADC chỉ ~40%).
6. **Vòng lặp Master 2000Hz (nâng từ 1000Hz):**
   - Giảm một nửa độ trễ pipeline sensor → PWM (tối đa 500µs thay vì 1ms): cho phép nâng Kp/Kd và tốc độ vào cua cao hơn mà vẫn không dao động lắc đuôi.
   - Khâu D vẫn cập nhật mỗi 2ms đúng như bản cũ — **thang tuning Kd/Kp cũ giữ nguyên**, không cần tune lại từ đầu.
   - Vận tốc Encoder cập nhật mỗi 2ms (trước đây 4ms): các quyết định phanh/xoay pivot phản ánh vận tốc thực nhanh gấp đôi.
7. **Vòng lặp kín vận tốc bánh xe (Wheel Closed-Loop FF+PI):**
   - Lớp điều khiển cascade: lái PD cho mục tiêu tốc độ từng bánh, PI (kèm Feedforward PWM) ép bánh đạt đúng tốc độ đo từ Encoder.
   - Chống windup tích phân, tự reset khi luồng phanh ngược/pivot ghi PWM thô, tự bypass khi Encoder chưa active.
8. **Telemetry không chặn vòng lặp (921600 baud):**
   - Ở 115200 cũ, mỗi dòng log ~100 ký tự chặn vòng điều khiển tới ~9ms (xe chạy 2m/s bị "mù" 17mm mỗi 100ms!). Nay 921600 baud + chu kỳ 200ms, dòng log vừa FIFO TX UART nên không block.
9. **Phát hiện Vạch Đích (Finish Line Detection):**
   - Tự động nhận diện vạch ngang đích (7..8 mắt cảm biến cùng đen liên tục $\ge 12\text{ms}$). Tự động phanh dừng và phát còi thông báo, bỏ qua vạch xuất phát trong 2 giây đầu.
10. **Khóa an toàn chống tự phóng (`STATE_SAFE_STOP`):**
    - Khi xe mất line quá 250ms (ngoài pha vạch nét đứt), xe tự động ngắt động cơ an toàn.
   - **Không tự động phóng đi** khi đặt lại lên line: Bắt buộc người điều khiển phải nhấn nút SW1 để xác nhận về chế độ `IDLE`.

---

## 5. Hướng dẫn Biên dịch & Nạp Code (PlatformIO)

Cài đặt tiện ích mở rộng PlatformIO trong VSCode:

```bash
# Biên dịch mã nguồn
pio run

# Nạp firmware vào ESP32
pio run -t upload

# Mở Serial Monitor để quan sát Telemetry (921600 baud)
pio run -t monitor
```
