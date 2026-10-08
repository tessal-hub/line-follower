# High-Speed ESP32 Line Tracking Car - Specs & Goal

## 1. Project Goal
- **Objective:** Pure speed benchmark line follower. Single continuous track with no intersections or branch logic.
- **Philosophy:** Zero network overhead (no WiFi, no BLE, no web server). Hard real-time control loop running on bare-metal Arduino/ESP32 framework.
- **Operation:** Direct physical button controls (`SW1` calib/e-stop, `SW2` start/stop, `SW3` speed modes / max power toggle).

---

## 2. Hardware Pinout (`config.h`)

| GPIO | Signal / Macro | Target Module / Pin | Mode | Description |
|---|---|---|---|---|
| **18** | `PIN_MOTOR_LEFT_IN1` | DRV8871 Left (IN1) | Output (PWM) | LEDC Ch 0, 20 kHz 8-bit |
| **19** | `PIN_MOTOR_LEFT_IN2` | DRV8871 Left (IN2) | Output (PWM) | LEDC Ch 1, 20 kHz 8-bit |
| **26** | `PIN_MOTOR_RIGHT_IN1` | DRV8871 Right (IN1) | Output (PWM) | LEDC Ch 2, 20 kHz 8-bit |
| **25** | `PIN_MOTOR_RIGHT_IN2` | DRV8871 Right (IN2) | Output (PWM) | LEDC Ch 3, 20 kHz 8-bit |
| **32** | `PIN_ENC_LEFT_A` | Left Encoder Phase A | `INPUT_PULLUP` | External interrupt (`CHANGE`) |
| **33** | `PIN_ENC_LEFT_B` | Left Encoder Phase B | `INPUT_PULLUP` | Direction detection |
| **34** | `PIN_ENC_RIGHT_A` | Right Encoder Phase A | `INPUT` | External interrupt (`CHANGE`) *(Needs ext 4.7k-10k pull-up)* |
| **35** | `PIN_ENC_RIGHT_B` | Right Encoder Phase B | `INPUT` | Direction detection *(Needs ext 4.7k-10k pull-up)* |
| **36** | `PIN_MUX_SIG` | 8-Ch MUX COM / Z | Analog ADC | ADC1_CH0 (`SENSOR_VP`), 12-bit ADC |
| **16** | `PIN_MUX_S0` | MUX Address Bit 0 | Output | Address line A |
| **17** | `PIN_MUX_S1` | MUX Address Bit 1 | Output | Address line B |
| **23** | `PIN_MUX_S2` | MUX Address Bit 2 | Output | Address line C |
| **21** | `PIN_I2C_SDA` | OLED SH1106 / SSD1306 | I2C Data | 400 kHz Fast I2C (Address `0x3C`) |
| **22** | `PIN_I2C_SCL` | OLED SH1106 / SSD1306 | I2C Clock | 400 kHz Fast I2C (Address `0x3C`) |
| **13** | `PIN_BTN_SW1` | Pushbutton SW1 | `INPUT_PULLUP` | Idle: Calibrate; Running: Emergency Stop |
| **14** | `PIN_BTN_SW2` | Pushbutton SW2 | `INPUT_PULLUP` | Toggle Run / Stop |
| **27** | `PIN_BTN_SW3` | Pushbutton SW3 | `INPUT_PULLUP` | Idle: Speed preset (Eco / Sport / Turbo Max); Run: Lap trigger |

---

## 3. Subsystem Architecture

### A. Motor Drivers (Dual DRV8871)
- **Motor Model:** JGB37-520 12V 1590RPM (`MOTOR_MODEL_INFO`)
- **Gear Reduction Ratio:** 6.25:1 (`GEAR_REDUCTION_RATIO = 6.25f`)
- **Motor Hall PPR:** 11 pulses/rev (`MOTOR_HALL_PPR = 11.0f`)
- **Encoder Ticks/Rev:** 68.75 ticks/wheel rev (`ENCODER_TICKS_PER_REV = 68.75f`)
- **Wheel Diameter:** 68 mm (`WHEEL_DIAMETER_MM = 68.0f`)
- **Max Motor RPM:** 1590 RPM (`MAX_MOTOR_RPM = 1590.0f`)
- **Minimum Starting PWM:** 135 / 255 (`MIN_START_PWM = 135`)
- **Power:** VM connected directly to 3S LiPo (~11.1V - 12.6V).
- **Control Mode:** Slow Decay (Drive / Brake) at 20 kHz carrier frequency.
- **Braking:** Actively shorts motor coils when braking to provide dynamic electromagnetic deceleration through sharp turns.

### B. Line Sensor Bar (8-Channel Multiplexer)
- **Multiplexer:** CD74HC4051 (or 74HC4067 using first 8 channels).
- **ADC Input:** GPIO 36 (ADC1_CH0, no conflict with WiFi/ADC2).
- **Sensor Weights:**
  - Ch 0 (`000`): Left edge (`-3.5`)
  - Ch 1 (`001`): Left mid-outer (`-2.5`)
  - Ch 2 (`010`): Left mid-inner (`-1.5`)
  - Ch 3 (`011`): Left center (`-0.5`)
  - Ch 4 (`100`): Right center (`+0.5`)
  - Ch 5 (`101`): Right mid-inner (`+1.5`)
  - Ch 6 (`110`): Right mid-outer (`+2.5`)
  - Ch 7 (`111`): Right edge (`+3.5`)

### C. Wheel Encoders (JGB37-520 12V Hall Effect)
- **Left:** GPIO 32 (Phase A) & GPIO 33 (Phase B) using internal pull-ups.
- **Right:** GPIO 34 (Phase A) & GPIO 35 (Phase B) input-only pins; external 4.7kΩ–10kΩ pull-ups to 3.3V mandatory.

### D. Display & User Input
- **OLED (SH1106 / SSD1306):** 128x64 display on I2C (`0x3C`) for calibration offsets, mode display, and lap timer. Minimal updates during run loop to maintain high loop frequencies (>500 Hz).
- **Buttons:** Hardware RC-filtered (10kΩ pull-up + 100nF capacitor to GND) on GPIO 13, 14, 27.

---

## 4. Power Distribution
1. **12V Rail (3S LiPo):** Direct feed to DRV8871 VM pins.
2. **5V Rail (Buck Converter):** Steps 12V down to 5V for ESP32 VIN / 5V pin and line sensor array.
3. **3.3V Rail (ESP32 LDO):** Supplies OLED and encoder pull-up resistors.
4. **Common Ground:** Battery (-), Buck GND, ESP32 GND, DRV8871 GND, and Sensor GND tied together to single low-impedance ground plane.
