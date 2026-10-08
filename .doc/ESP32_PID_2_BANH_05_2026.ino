#include <EEPROM.h>
#include <Arduino.h>
#define RDIR 18     // chiều quay động cơ phải
#define RPWM 21     // tốc độ động cơ phải
#define LDIR 17     // chiều quay động cơ trái
#define LPWM 19     // tốc độ động cơ trái
#define BUTTON1 14  // nút nhấn 1
#define BUTTON2 27  // nút nhấn 2
#define BUTTON3 12  // nút nhấn 3
#define BUZZER 13   // chân điều khiển loa
#define RED 5       // Chân RED trên mạch
#define GREEN 2     // Chân GREEN trên mạch
#define BLUE 15     // Chân BLUE trên mạch
#define EEPROM_SIZE 40
hw_timer_t* timer = NULL;  //khơi tạo timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
//---------------------------------------------------
int addr = 0;
volatile long lastPos;
volatile unsigned char isCalib = 0;
int servoPwm;
volatile int calPID = 0;
volatile unsigned char sensor;
unsigned int sensorValue[8];
unsigned int sensorPID[8];
unsigned int black_value[8];
unsigned int white_value[8];
unsigned int compare_value[8];
int speed_run_forward;
int nut1 = 0;
int nut2 = 2000;
int nut3 = 3000;


int vitri = 0;
volatile long posPID;
volatile int cnt = 0;
volatile int cnt1 = 0;
volatile int cnt2 = 0;
volatile int cnt3 = 0;
volatile int cnt4 = 0;
unsigned char pattern, start, trangThai;
int line = 0;
int RememberLine = 0;
volatile float kp;
volatile int kd;
volatile int PIDfre = 0;
void setup() {


  //---------------PWM------------------------//
  pinMode(LDIR, OUTPUT);
  pinMode(RDIR, OUTPUT);
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  digitalWrite(RDIR, LOW);
  digitalWrite(LDIR, LOW);
  ledcAttach(LPWM, 2000, 12);
  ledcAttach(RPWM, 2000, 12);
  digitalWrite(RED, 1);
  digitalWrite(GREEN, 1);
  digitalWrite(BLUE, 1);
  //-------------------------------------------------//
  //CÀI ĐẶT CÁC THÔNG SỐ BAN ĐẦU
  speed_run(0, 0);  // dừng 2 bánh
  trangThai = 9;    // set trạng thái chạy
  pattern = 11;
  start = 0;  // Trước khi chạy học màu lại, đưa cảm biến qua lại trên đường line rồi bấm SW1 để lưu
  EEPROM.begin(EEPROM_SIZE);
  readEeprom();
  Serial.begin(115200);
  //-------------------------------------------------//
  // for (int i = 2000; i < 3500; i += 500) {
  //   tone(BUZZER, i, 100);
  //   delay(100);
  // }
  timer_init();
  isCalib = 0;
  pinMode(BUZZER, OUTPUT);
  beep(50);
  //-------------------------------------------------//
}
void timer_init() {
  //timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 1000000, true, 999000);
}
void PID() {
  if (calPID == 1) {
    long sum = 0;
    long avg = 0;
    long i, iP, iD;
    long iRet;

    for (int j = 0; j < 8; j = j + 1) {
      avg = (avg + (sensorPID[j]) * ((j)*3000));
      sum = sum + sensorPID[j];
    }
    i = ((avg / sum) - 10500);
    posPID = i;
    kp = 1;
    kd = 15;
    iP = kp * i;
    iD = kd * (lastPos - i);
    iRet = (iP - iD);
    if ((iRet < -12000)) {
      iRet = 0;
    }
    servoPwm = iRet / 5;
    //  Serial.println(servoPwm);
    lastPos = i;
    calPID = 0;
  }
}
void read_sensor()  // hàm đọc cảm biến
{
  unsigned char temp = 0;
  PIDfre = PIDfre + 1;
  //isReadingSensor = 1;
  sensorValue[0] = 4095 - analogRead(36);
  sensorValue[1] = 4095 - analogRead(39);
  sensorValue[2] = 4095 - analogRead(34);
  sensorValue[3] = 4095 - analogRead(35);
  sensorValue[4] = 4095 - analogRead(32);
  sensorValue[5] = 4095 - analogRead(33);
  sensorValue[6] = 4095 - analogRead(25);
  sensorValue[7] = 4095 - analogRead(26);
  for (int j = 0; j < 8; j = j + 1) {
    if (isCalib == 0) {
      if (sensorValue[j] < black_value[j])
        sensorValue[j] = black_value[j];
      if (sensorValue[j] > white_value[j])
        sensorValue[j] = white_value[j];
      sensorPID[j] = map(sensorValue[j], black_value[j], white_value[j], 1, 3000);
    }
    temp = temp << 1;
    if (sensorValue[j] > compare_value[j]) {
      temp |= 0x01;
    } else {
      temp &= 0xfe;
    }
    sensor = temp;
  }
  if (PIDfre == 2) {
    PIDfre = 0;
    calPID = 1;
  }
}
void ARDUINO_ISR_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);  //vào chế độ tránh xung đột
  cnt = cnt + 1;
  cnt1 = cnt1 + 1;
  read_sensor();
  portEXIT_CRITICAL_ISR(&timerMux);  // thoát
}
void loop()  // vòng lặp, không dùng delay() trong này
{
  PID();

  while (start == 0)  // nhấn nút để chạy
  {

    PID();
    if (isCalib == 0) {

      if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để học màu
        for (int i = 0; i < 8; i = i + 1) {
          black_value[i] = 5000;
        }
        for (int i = 0; i < 8; i = i + 1) {
          white_value[i] = 0;
        }
        for (int i = 0; i < 8; i = i + 1) {
          compare_value[i] = 5000;
        }
        beep(100);
        while (digitalRead(BUTTON1) == 0) {
        }
        isCalib = 1;
        break;
      }
      if (digitalRead(BUTTON2) == 0) {  // Nhấn SW2 để chạy
        beep(100);
        start = 1;  // chạy luôn, không cần học màu lại
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut2;  // chậm
        break;
      }
      if (digitalRead(BUTTON3) == 0) {  // Nhấn SW3 để chạy
        beep(100);
        start = 1;  // chạy luôn, không cần học màu lại
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut3;  // nhanh
        break;
      }
    } else {
      updateLine();
      if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để chạy
        for (int i = 0; i < 8; i = i + 1) {
          // compare_value[i] = (black_value[i] + white_value[i]) / 2;
          EEPROM.write(2 * i, black_value[i] / 16);
          EEPROM.write((2 * i) + 1, black_value[i] % 16);
          delay(1);
        }
        for (int i = 0; i < 8; i = i + 1) {
          // compare_value[i] = (black_value[i] + white_value[i]) / 2;
          EEPROM.write(16 + (2 * i), white_value[i] / 16);
          EEPROM.write(17 + (2 * i), white_value[i] % 16);
          //         EEPROM.write(8 + i, white_value[i] / 4);
          delay(1);
        }
        EEPROM.commit();
        beep(300);

        while (digitalRead(BUTTON1) == 0) {
        }
        start = 1;  // học màu xong va bắt đầu chạy
        isCalib = 0;
        speed_run_forward = 0;  //test PID
      }
    }
  }

  switch (trangThai) {  // đoạn mã trạng thái máy
    case 9:
      if (digitalRead(BUTTON1) == 0) {  // Nhấn SW1 để test PID
        beep(100);
        trangThai = 91;
        cnt = 0;
        speed_run_forward = nut1;  //test PID
      }
      if (digitalRead(BUTTON2) == 0) {  // Nhấn SW2 để chạy, tốc độ nut2 ở phía trên
        beep(100);
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut2;  // chậm
      }
      if (digitalRead(BUTTON3) == 0) {  // Nhấn SW3 để chạy, tốc độ nut3 ở phía trên
        beep(100);
        trangThai = 10;
        cnt = 0;
        cnt1 = 0;
        speed_run_forward = nut3;  // nhanh
      }
      break;
    case 91:  //test PID
      handleAndSpeed(servoPwm, 0);
      break;
    case 10:
      trangThai = 11;
      break;
    case 11:
      PID();
      runforwardline(speed_run_forward);
      break;
    case 12:
      break;
    case 13:
      break;
    case 14:
      break;
    case 100:
      speed_run(0, 0);
      break;
    default:
      trangThai = 11;
      break;
  }
}
///////////////////////////////////////////////////////////////////////
void runforwardline(int tocdo)  // hàm chạy bám line
{
  PID();

  switch (pattern) {  // đoạn mã trạng thái máy
    case 10:
      /*if (cnt >= speed_run_forward) {  // KHởi động mềm
        pattern = 11;
        cnt = 0;
        break;
      }*/
      //  if (cnt >= 50) {
      pattern = 11;
      //     cnt = 0;
      break;
      //   }
      handleAndSpeed(servoPwm, tocdo);
      break;
    case 11:
      // if (sensorMask(0x81) == 0x81) {
      //   pattern = 100;
      //    break;
      //  }
      if (sensorMask(0x01) == 0x01) {
        RememberLine = 1;
        //  cnt = 0;
      } else if (sensorMask(0x80) == 0x80) {
        RememberLine = -1;
        //  cnt = 0;
      }
      //if (cnt > 500) RememberLine = 0;
      if (sensor == 0b00000000) {  // nếu bị mất line do cua gấp và có biến nhớ thì nhảy vô trường hợp giải quyết cua gấp
        if (cnt1 > 40000) {        // Dừng chạy sau 40 giây
          trangThai = 100;
          pattern = 100;
        } else if (RememberLine != 0) {
          if (RememberLine == 1) {
            // cua = cua + 1;
            speed_run(1600, -100);
            pattern = 12;
          } else if (RememberLine == -1) {
            //  cua = cua + 1;
            speed_run(-100, 1600);
            pattern = 12;
          }                 // chuyển trạng thái sang 12 để xử lý cua gấp
        } else {            // Nếu mất line mà không có biến nhớ thì chạy thẳng, đoạn line đứt khúc
          speed_run(0, 0);  // chạy thẳng
        }
        break;
      } else {
        switch (sensor) {
          case 0b00011000:
          case 0b00111100:
          case 0b00011100:
          case 0b00001000:
          case 0b00001100:
          case 0b00011110:
          case 0b00000100:
          case 0b00001110:
          case 0b00000110:
          case 0b00001111:
          case 0b00011111:
          case 0b00111111:
          case 0b01111111:
          case 0b01111100:
          case 0b01111110:
          case 0b00111110:
          case 0b00000010:
          case 0b00000111:
          case 0b00000011:
          case 0b00000001:
          case 0b00000000:
          case 0b00010000:
          case 0b00111000:
          case 0b00110000:
          case 0b01111000:
          case 0b00100000:
          case 0b01110000:
          case 0b01100000:
          case 0b11110000:
          case 0b11111000:
          case 0b11111100:
          case 0b11111110:
          case 0b01000000:
          case 0b11100000:
          case 0b11000000:
          case 0b10000000:
            //isIMG = 0;
            handleAndSpeed(servoPwm, tocdo);
            break;
          default:
            //isIMG = 1;
            handleAndSpeed(servoPwm, tocdo);
            break;
        }
      }  //handleAndSpeed(servoPwm, tocdo);  // Nếu không có gì xảy ra thì bám line bằng PID
      // if (sensorMask(0b00011000) != 0b00000000) {
      //   RememberLine = 0;
      // }
      break;
    case 12:
      if (RememberLine == 1) {
        speed_run(1600, -100);
        pattern = 21;
        break;
      } else if (RememberLine == -1) {
        speed_run(-100, 1600);
        pattern = 31;
        break;
      } else {
        pattern = 11;
        break;
      }
    case 21:
      speed_run(1600, -100);
      RGB(1);
      if (sensorMask(0x03) != 0) {
        speed_run(1600, 800);
        pattern = 22;
      }
      break;
    case 22:
      speed_run(1600, 800);
      RGB(1);
      if (sensorMask(0xfc) != 0) {
        //if ((sensor == 0x03) || (sensor == 0x06) || (sensor == 0x0c) || (sensor == 0x18) || (sensor == 0x30) || (sensor == 0x60) || (sensor == 0x30) || (sensor == 0xc0)) {
        pattern = 11;
        RememberLine = 0;
        cnt = 0;
        // line = 0;
      }
      break;


    case 31:
      speed_run(-100, 1600);
      RGB(2);
      if (sensorMask(0xc0) != 0) {
        speed_run(800, 1600);
        pattern = 32;
      }
      break;
    case 32:
      speed_run(800, 1600);
      RGB(2);
      if (sensorMask(0x3f) != 0) {
        pattern = 11;
        RememberLine = 0;
        cnt = 0;
        // line = 0;
      }
      break;
    case 100:
      speed_run(0, 0);
      RGB(millis() / 100 % 3);
      break;
    default:
      break;
  }
}
//--------------------------------------------------------------------------//
void updateLine() {
  // Serial.println(sensor, BIN);
  //if (sensor == 0xff) beep(100);
  for (int i = 0; i < 8; i = i + 1) {
    Serial.print(sensorValue[i]);
    Serial.print("  ");
    delay(1);
    if (black_value[i] == 0) black_value[i] = 5000;
    if ((sensorValue[i]) < black_value[i]) black_value[i] = sensorValue[i];
    if (sensorValue[i] > white_value[i]) white_value[i] = sensorValue[i];
    compare_value[i] = ((black_value[i] + white_value[i]) / 2);
  }
  Serial.println();
  RGB(millis() / 100 % 3);
}
void RGB(int color) {
  switch (color) {
    case 0:
      digitalWrite(RED, 0);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
    case 1:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 0);
      digitalWrite(BLUE, 1);
      break;
    case 2:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 0);
      break;
    default:
      digitalWrite(RED, 1);
      digitalWrite(GREEN, 1);
      digitalWrite(BLUE, 1);
      break;
  }
}
void readEeprom() {
  for (int i = 0; i < 8; i = i + 1) {
    black_value[i] = EEPROM.read(i * 2) * 16 + EEPROM.read((i * 2) + 1);
  }
  for (int i = 0; i < 8; i = i + 1) {
    white_value[i] = EEPROM.read(16 + (2 * i)) * 16 + EEPROM.read(17 + (2 * i));
  }
  for (int i = 0; i < 8; i = i + 1) {
    compare_value[i] = ((black_value[i] + white_value[i]) / 2);
  }
}
//-------------------------------------------------------------------//

//---------------------------------------------------------------------------//
void handleAndSpeed(int angle, int speed1) {
  int speedLeft;
  int speedRight;
  if ((speed1 + angle) > 4095) {
    speed1 = 4095 - angle;
  }
  if ((speed1 - angle) > 4095) {
    speed1 = 4095 + angle;
  }
  speedLeft = (speed1 + angle);
  speedRight = (speed1 - angle);
  speed_run(speedLeft, speedRight);
}
//---------------------------------------------------------------------------//
void speed_run(int speedDC_left, int speedDC_right)  // hàm truyền vào tốc độ động cơ trái + phải
{
  if (speedDC_left < -4095) speedDC_left = -4095;
  if (speedDC_right < -4095) speedDC_right = -4095;
  if (speedDC_left > 4095) speedDC_left = 4095;
  if (speedDC_right > 4095) speedDC_right = 4095;
  //if(isReadingSensor == 0){
  if (speedDC_left < 0) {
    analogWrite(LPWM, 4095 + speedDC_left);
    digitalWrite(LDIR, HIGH);
  } else {
    //speedDC_left = speedDC_left;
    analogWrite(LPWM, speedDC_left);
    digitalWrite(LDIR, LOW);
  }
  if (speedDC_right < 0) {
    analogWrite(RPWM, 4095 + speedDC_right);
    digitalWrite(RDIR, HIGH);
  } else {
    //speedDC_right = speedDC_right;
    analogWrite(RPWM, speedDC_right);
    digitalWrite(RDIR, LOW);
  }
  // }
}
//-----------------------------------------------------------//
void beep(int timer) {
  digitalWrite(BUZZER, 1);
  delay(timer);
  digitalWrite(BUZZER, 0);
}
unsigned char sensorMask(unsigned char mask) {
  return (sensor & mask);
}
