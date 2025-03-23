#include <Stepper.h>

const int stepsPerRevolution = 2048; // 모든 모터에 대해 동일한 스텝 수 사용
const int motorCount = 10;

// 예시 핀 배정 (각 모터 4핀씩)
// 실제 사용 환경에 맞게 수정하세요.
int motorPins[motorCount][4] = {
  {A0, A2, A1, A3},       // L1
  {A4, A6, A5, A7},       // L2
  {A8, A10, A9, A11},      // L3
  {A12, A14, A13, A15},    // L4
  {23, 27, 25, 29},        // L5
  {4, 6, 5, 7},           // R1
  {8, 10, 9, 11},         // R2
  {14, 16, 15, 17},       // R3
  {18, 20, 19, 21},       // R4
  {22, 26, 24, 28}        // R5
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
  // 시리얼 데이터를 모두 읽어들임
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    Serial.print("Received input: ");
    Serial.println(input);

    // "finish" 명령 처리: 모든 모터를 초기 위치로 복귀
    if (input.equalsIgnoreCase("finish")) {
      for (int i = 0; i < motorCount; i++) {
        motors[i]->step(currentPositions[i]); // 누적 스텝만큼 반대방향 회전
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
  
  // 모든 명령이 수신되면, 각 모터에 대해 동시에 동작 실행
  if (allReceived) {
    // 각 모터의 이동해야 할 스텝 수(양의 정수)를 계산합니다.
    // REV: motors[i]->step(1) 호출, 이동 후 currentPositions[i] -= 1;
    // FWD: motors[i]->step(-1) 호출, 이동 후 currentPositions[i] += 1;
    long remainingSteps[motorCount];
    for (int i = 0; i < motorCount; i++) {
      String cmd = commands[i].command;
      float errorAngle = commands[i].errorAngle;
      // 360°가 stepsPerRevolution 스텝에 해당하므로 회전할 스텝 수 계산
      long steps = (long)((errorAngle / 360.0) * stepsPerRevolution);
      if (cmd.equalsIgnoreCase("STOP")) {
        remainingSteps[i] = 0;
      } else {
        remainingSteps[i] = steps; // REV, FWD 모두 동일한 스텝 수(양의 값)
      }
    }
    
    // 남은 스텝 수가 0이 될 때까지 모든 모터를 1스텝씩 번갈아 가며 실행합니다.
    bool motorsActive = true;
    while (motorsActive) {
      motorsActive = false;
      for (int i = 0; i < motorCount; i++) {
        if (remainingSteps[i] > 0) {
          if (commands[i].command.equalsIgnoreCase("REV")) {
            motors[i]->step(1);
            currentPositions[i] -= 1;
          } else if (commands[i].command.equalsIgnoreCase("FWD")) {
            motors[i]->step(-1);
            currentPositions[i] += 1;
          }
          remainingSteps[i]--;
          if (remainingSteps[i] > 0) {
            motorsActive = true;
          }
        }
      }
      delay(10); // 각 스텝 사이의 지연 시간 (속도 조절 필요시 수정)
    }
    // 모든 모터의 동작이 완료되면 done 신호 전송
    Serial.println("done");
    // 명령 배열 초기화
    for (int i = 0; i < motorCount; i++) {
      commands[i].valid = false;
    }
  }
}
