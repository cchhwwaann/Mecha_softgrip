#include <Stepper.h>

const int stepsPerRevolution = 2048; // 모든 모터에 대해 동일한 스텝 수 사용
const int motorCount = 10;

// 예시 핀 배정 (각 모터 4핀씩)
// 실제 사용 환경에 맞게 수정하세요.
int motorPins[motorCount][4] = {
  {2, 3, 4, 5},       // L1
  {8, 10, 9, 11},     // L2
  {6, 7, 12, 13},     // L3
  {A0, A1, A2, A3},   // L4
  {A4, A5, A6, A7},   // L5
  {22, 23, 24, 25},   // R1
  {26, 27, 28, 29},   // R2
  {30, 31, 32, 33},   // R3
  {34, 35, 36, 37},   // R4
  {38, 39, 40, 41}    // R5
};

Stepper* motors[motorCount];
long currentPositions[motorCount] = {0,0,0,0,0,0,0,0,0,0};

// 구조체를 이용하여 각 모터에 대한 명령(CSV 데이터)을 저장
struct MotorCommand {
  bool valid;
  String command;   // "REV", "FWD", "STOP"
  float errorAngle; // 에러 각도 (°)
};
MotorCommand commands[motorCount];  // 인덱스 0~4: L1~L5, 5~9: R1~R5

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // 시리얼 모니터가 연결될 때까지 대기

  // 각 모터 객체 생성 및 초기 설정 (예: 속도 14 RPM)
  for (int i = 0; i < motorCount; i++) {
    motors[i] = new Stepper(stepsPerRevolution,
                            motorPins[i][0],
                            motorPins[i][1],
                            motorPins[i][2],
                            motorPins[i][3]);
    motors[i]->setSpeed(14);
  }
}

void loop() {
  // 읽은 모든 시리얼 데이터를 처리
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.print("Received input: ");
    Serial.println(input);

    // "finish" 명령 처리: 모든 모터를 초기 위치로 복귀
    if (input.equalsIgnoreCase("finish")) {
      for (int i = 0; i < motorCount; i++) {
        motors[i]->step(-currentPositions[i]); // 누적 스텝만큼 반대방향 회전
        currentPositions[i] = 0;
      }
      Serial.println("done");
      // 명령 배열 초기화
      for (int i = 0; i < motorCount; i++) {
        commands[i].valid = false;
      }
      continue;
    }

    // CSV 형식: 예 "L3,REV,45.00" 또는 "R5,FWD,10.50"
    // 첫 글자가 모터 식별자 ('L' 또는 'R')이고, 그 다음 숫자가 1~5여야 함.
    char motorChar = input.charAt(0);
    int motorNum = input.charAt(1) - '0'; // 1~5
    int motorIndex = -1;
    if (motorChar == 'L') {
      motorIndex = motorNum - 1;  // L1 -> 0, L2 -> 1, ... L5 -> 4
    } else if (motorChar == 'R') {
      motorIndex = 5 + (motorNum - 1);  // R1 -> 5, R2 -> 6, ... R5 -> 9
    } else {
      Serial.println("Error: Invalid motor identifier.");
      continue;
    }

    // CSV 구분자 처리
    int firstComma = input.indexOf(',');
    int secondComma = input.indexOf(',', firstComma + 1);
    if (firstComma < 0 || secondComma < 0) {
      Serial.println("Error: Invalid CSV format.");
      continue;
    }
    String cmd = input.substring(firstComma + 1, secondComma);
    String errorStr = input.substring(secondComma + 1);
    float errorAngle = errorStr.toFloat();

    // 해당 모터의 명령을 저장
    commands[motorIndex].valid = true;
    commands[motorIndex].command = cmd;
    commands[motorIndex].errorAngle = errorAngle;
  } // end while Serial.available()

  // 모든 10개 모터의 명령이 수신되었는지 확인
  bool allReceived = true;
  for (int i = 0; i < motorCount; i++) {
    if (!commands[i].valid) { 
      allReceived = false;
      break;
    }
  }
  if (allReceived) {
    // 모든 모터에 대해 명령 실행
    for (int i = 0; i < motorCount; i++) {
      String cmd = commands[i].command;
      float errorAngle = commands[i].errorAngle;
      // 360°가 stepsPerRevolution 스텝에 해당하므로 회전할 스텝 수 계산
      long steps = (long)((errorAngle / 360.0) * stepsPerRevolution);

      if (cmd.equalsIgnoreCase("STOP")) {
        // 움직임 없음
      }
      else if (cmd.equalsIgnoreCase("REV")) {
        motors[i]->step(-steps);
        currentPositions[i] -= steps;
      }
      else if (cmd.equalsIgnoreCase("FWD")) {
        motors[i]->step(steps);
        currentPositions[i] += steps;
      }
      else {
        Serial.print("Error: Unknown command for motor ");
        if (i < 5)
          Serial.print("L");
        else
          Serial.print("R");
        Serial.println(i < 5 ? i + 1 : i - 4);
      }
    }
    // 모든 모터 명령 실행 후 단 한 번 "done" 전송
    Serial.println("done");
    // 명령 배열 초기화
    for (int i = 0; i < motorCount; i++) {
      commands[i].valid = false;
    }
  }
}
