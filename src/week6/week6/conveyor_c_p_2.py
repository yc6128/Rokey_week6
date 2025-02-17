// 기존 코드 일부
typedef enum _CONVEYOR_STATE { Conveyor_Ready,
                               Conveyor_Run } CONVEYOR_STATE;

CONVEYOR_STATE state = Conveyor_Ready;
unsigned long step_count = 0;

void loop() {
  unsigned long time_c = millis();

  // 시리얼로 명령 수신 처리 및 상태 전환 등 기존 로직 ...
  if (Serial.available() > 0) {
    int incomingByte = Serial.read();
    // (여기서는 명령을 받아서 state와 step_count를 설정)
    if (incomingByte == 's') {
      // 예: 정지 명령을 받았다고 가정
      state = Conveyor_Ready;
    } else {
      state = Conveyor_Run;
      // step_count 설정 로직
    }
  }

  step_run(time_c);

  // 상태 메시지 전송 (예시)
  if (state == Conveyor_Run) {
    Serial.print("Status:Running, Moved:");
    Serial.println(step_count);
  } else {
    Serial.println("Status:Ready");
  }
  
  delay(200);  // 너무 잦은 업데이트는 피하기 위해 약간의 딜레이 추가
}
