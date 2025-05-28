#include <SoftwareSerial.h>

SoftwareSerial bleSerial(A0, A1); // RX, TX (BLE 통신용)

// BLE 센서 수신 관련 변수
String uartString = "";
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 1000;
int ble_bat = 0, ble_roll = 0, ble_pitch = 0, ble_height = 0;

//==== 드론 2호 제어에 필요한 변수 선언 부분 시작 ====
const unsigned char startBit_1 = 0x26;
const unsigned char startBit_2 = 0xa8;
const unsigned char startBit_3 = 0x14;
const unsigned char startBit_4 = 0xb1;
const unsigned char len = 0x14;
unsigned char checkSum = 0;

int roll = 0;
int pitch = 0;
int yaw = 0;
int throttle = 0;
int option = 0x000f;

int p_vel = 0x0064;
int y_vel = 0x0064;

unsigned char drone_action = 0;
unsigned char payload[14];
unsigned int firstRoll = 0;
unsigned int firstPitch = 0;
//===== 드론 2호 제어에 필요한 변수 선언 부분 끝 =====

// BLE 센서 출력 시작 명령
void startOutputCommand() {
  bleSerial.print("at+writeh000d26a814b2080a0901\r");
}

// HEX 문자 2개 → 10진수 변환
int hexToDecimal(char high, char low) {
  char hexStr[3] = { high, low, '\0' };
  return (int) strtol(hexStr, NULL, 16);
}

//========= 드론 제어 함수들 =========

// 드론 높이 제어
void checkThrottle() {
  if (!digitalRead(6)) {
    if (throttle > 9)
      throttle -= 10;
  } else if (!digitalRead(5)) {
    if (throttle < 141)
      throttle += 20;
  }
}

// 드론 좌/우 회전 제어
void checkYaw() {
  if (throttle == 0)
    yaw = 0;

  if (!digitalRead(7)) {
    if (yaw > -170)
      yaw -= 10;
  } else if (!digitalRead(8)) {
    if (yaw < 170)
      yaw += 10;
  }
}

// 비상 착륙 제어
void checkEmergency() {
  if (!digitalRead(9)) {
    roll = 0;
    pitch = 0;
    yaw = 0;
    throttle = 0;
    option = 0x000e;
  } else {
    option = 0x000f;
  }
}

// 좌/우 이동 제어
void checkRoll() {
  unsigned int secondRoll = analogRead(4);

  if (secondRoll < firstRoll - 450)
    roll = -200;
  else if (secondRoll < firstRoll - 350)
    roll = -160;
  else if (secondRoll < firstRoll - 250)
    roll = -120;
  else if (secondRoll < firstRoll - 150)
    roll = -80;
  else if (secondRoll < firstRoll - 50)
    roll = -40;
  else if (secondRoll < firstRoll + 50)
    roll = 0;
  else if (secondRoll < firstRoll + 150)
    roll = 40;
  else if (secondRoll < firstRoll + 250)
    roll = 80;
  else if (secondRoll < firstRoll + 350)
    roll = 120;
  else if (secondRoll < firstRoll + 450)
    roll = 160;
  else
    roll = 200;
}

// 전/후진 이동 제어
void checkPitch() {
  unsigned int secondPitch = analogRead(5);

  if (secondPitch < firstPitch - 450)
    pitch = -200;
  else if (secondPitch < firstPitch - 350)
    pitch = -160;
  else if (secondPitch < firstPitch - 250)
    pitch = -120;
  else if (secondPitch < firstPitch - 150)
    pitch = -80;
  else if (secondPitch < firstPitch - 50)
    pitch = -40;
  else if (secondPitch < firstPitch + 50)
    pitch = 0;
  else if (secondPitch < firstPitch + 150)
    pitch = 40;
  else if (secondPitch < firstPitch + 250)
    pitch = 80;
  else if (secondPitch < firstPitch + 350)
    pitch = 120;
  else if (secondPitch < firstPitch + 450)
    pitch = 160;
  else
    pitch = 200;
}

// 데이터 전송시 체크섬 계산
void checkCRC() {
  memset(payload, 0x00, 14);

  payload[0] = roll & 0x00ff;
  payload[1] = (roll >> 8) & 0x00ff;
  payload[2] = pitch & 0x00ff;
  payload[3] = (pitch >> 8) & 0x00ff;
  payload[4] = yaw & 0x00ff;
  payload[5] = (yaw >> 8) & 0x00ff;
  payload[6] = throttle & 0x00ff;
  payload[7] = (throttle >> 8) & 0x00ff;
  payload[8] = option & 0x00ff;
  payload[9] = (option >> 8) & 0x00ff;
  payload[10] = p_vel & 0x00ff;
  payload[11] = (p_vel >> 8) & 0x00ff;
  payload[12] = y_vel & 0x00ff;
  payload[13] = (y_vel >> 8) & 0x00ff;

  checkSum = 0;
  for (int i = 0; i < 14; i++)
    checkSum += payload[i];

  checkSum &= 0x00ff;
}

// 드론 제어 명령 전송
void sendDroneCommand() {
  bleSerial.print("at+writeh000d");
  
  if (startBit_1 < 0x10) bleSerial.print("0");
  bleSerial.print(String(startBit_1, HEX));

  if (startBit_2 < 0x10) bleSerial.print("0");
  bleSerial.print(String(startBit_2, HEX));

  if (startBit_3 < 0x10) bleSerial.print("0");
  bleSerial.print(String(startBit_3, HEX));

  if (startBit_4 < 0x10) bleSerial.print("0");
  bleSerial.print(String(startBit_4, HEX));

  if (len < 0x10) bleSerial.print("0");
  bleSerial.print(String(len, HEX));

  if (checkSum < 0x10) bleSerial.print("0");
  bleSerial.print(String(checkSum, HEX));

  for (int i = 0; i < 14; i++) {
    if (payload[i] < 0x10) bleSerial.print("0");
    bleSerial.print(String(payload[i], HEX));
  }

  bleSerial.print("\r");
  delay(50);
}

// 드론 제어 명령 데이터 첫 진행 처리
unsigned char startDroneControl() {
  if (!digitalRead(10)) {
    firstRoll = analogRead(4);
    firstPitch = analogRead(5);
    drone_action = 1;
  }
  return drone_action;
}

// BLE 시리얼 데이터 처리
void handleBleSerial() {
  while (bleSerial.available()) {
    char inChar = bleSerial.read();
    uartString += inChar;

    if (uartString.length() > 2 && uartString.endsWith("\r\n")) {
      // 예: "\r\n000D,26A814A212...."
      if (uartString.startsWith("\r\n000D,26A814A212") && uartString.length() > 44) {
        ble_bat    = hexToDecimal(uartString[25], uartString[26]);
        ble_roll   = hexToDecimal(uartString[29], uartString[30]);
        ble_pitch  = hexToDecimal(uartString[31], uartString[32]);
        ble_height = hexToDecimal(uartString[33], uartString[34]);

        Serial.print("BAT : "); Serial.print(ble_bat);
        Serial.print(", Roll : "); Serial.print(ble_roll);
        Serial.print(", Pitch : "); Serial.print(ble_pitch);
        Serial.print(", Height : "); Serial.println(ble_height);
      }
      uartString = "";
    }
  }
}

//========= SETUP 시작 =========
void setup() {
  Serial.begin(9600);
  Serial.println("Drone + BLE Sensor Start");

  bleSerial.begin(9600);

  // 입력 핀 풀업 설정
  for (int i = 5; i <= 10; i++) {
    pinMode(i, INPUT_PULLUP);
  }

  delay(500);
  bleSerial.print("at+writeh000e0100\r"); // 초기화 명령
  delay(100);
  startOutputCommand(); // 센서 출력 시작
}
//========= SETUP 끝 ==========

unsigned long Check = 0;
const unsigned long Interval = 1;

void loop() {
  if (millis() - Check >= Interval) {
    handleBleSerial();
    Check = millis();
  }

  if (startDroneControl()) {
    checkThrottle();
    checkRoll();
    checkPitch();
    checkYaw();
    checkEmergency();
    checkCRC();
    sendDroneCommand();
  }
}
