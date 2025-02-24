#include <Stepper.h>

const int stepsPerRevolution = 2048; 
// ULN2003 보드와 28BYJ-48 모터의 일반적인 연결:
// IN1 -> 8, IN2 -> 10, IN3 -> 9, IN4 -> 11
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

long currentPosition = 0;  // 모터의 현재 누적 스텝 (0이 초기 위치)

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // 시리얼 모니터가 연결될 때까지 대기
  Serial.println("Stepper motor (L2) test started.");
  myStepper.setSpeed(14); // RPM 단위 속도 설정
  Serial.println("L2 모터 제어를 위한 CSV 명령을 기다립니다.");
  Serial.println("예) L2,REV,45.00 또는 L2,FWD,10.50 또는 L2,STOP,0.00");
  Serial.println("또한 'r' 또는 'R'을 입력하면 모터가 초기 위치(0도)로 돌아갑니다.");
}

void loop() {
  if (Serial.available() > 0) {
    // 개행문자('\n')까지 읽어 한 줄의 문자열을 가져옴
    String input = Serial.readStringUntil('\n');
    input.trim(); // 앞뒤 공백 제거

    // 디버깅 메시지 출력
    Serial.print("Received input: ");
    Serial.println(input);

    // 글로벌 정지 명령 "0"이 수신되면 모터 정지 (현재 위치 유지)
    if (input.equals("0")) {
      Serial.println("Global stop command received. Stopping motor L2.");
      Serial.println("done");
      return;
    }

    // "r" 또는 "R" 입력 시 모터를 초기 위치(0도)로 복귀
    if (input.equalsIgnoreCase("r")) {
      Serial.println("Return command received. Returning motor L2 to initial position (0°).");
      myStepper.step(-currentPosition); // 현재 누적 스텝 반대로 회전
      currentPosition = 0;
      Serial.println("Motor L2 returned to initial position (0°).");
      Serial.println("done");
      return;
    }

    // 입력 문자열이 "L2,"로 시작하는지 확인 (제어할 모터는 L2)
    if (input.startsWith("L2,")) {
      // CSV 형식: L2,command,error_value
      int firstComma = input.indexOf(',');
      int secondComma = input.indexOf(',', firstComma + 1);
      if (firstComma >= 0 && secondComma >= 0) {
        String command = input.substring(firstComma + 1, secondComma);
        String errorStr = input.substring(secondComma + 1);
        float errorAngle = errorStr.toFloat();  // 에러 각도 (°)

        Serial.print("Command for L2: ");
        Serial.print(command);
        Serial.print(" with error angle: ");
        Serial.println(errorAngle);

        // 에러 각도를 기반으로 회전 스텝 계산 (360°에 2048스텝)
        long steps = (long)((errorAngle / 360.0) * stepsPerRevolution);

        // 명령에 따라 모터 회전 (REV: 역방향, FWD: 정방향, STOP: 회전 없음)
        if (command.equalsIgnoreCase("STOP")) {
          Serial.println("STOP command received. No movement.");
          Serial.println("done");
        }
        else if (command.equalsIgnoreCase("REV")) {
          Serial.print("Reversing motor L2 by ");
          Serial.print(steps);
          Serial.println(" steps.");
          myStepper.step(-steps);
          currentPosition -= steps;
          Serial.print("Current motor L2 position (steps): ");
          Serial.println(currentPosition);
          Serial.println("done");
        }
        else if (command.equalsIgnoreCase("FWD")) {
          Serial.print("Moving motor L2 forward by ");
          Serial.print(steps);
          Serial.println(" steps.");
          myStepper.step(steps);
          currentPosition += steps;
          Serial.print("Current motor L2 position (steps): ");
          Serial.println(currentPosition);
          Serial.println("done");
        }
        else {
          Serial.println("Error: Unknown command received.");
        }
      } else {
        Serial.println("Error: Invalid CSV format.");
      }
    }
    else {
      Serial.println("Error: Command not for L2. Ignored.");
    }
  }
}
