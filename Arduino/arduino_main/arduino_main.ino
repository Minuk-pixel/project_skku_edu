#include <Car_Library.h>

#define STEER_IN1 12
#define STEER_IN2 13
#define LEFT_IN1 11
#define LEFT_IN2 10
#define RIGHT_IN1 9
#define RIGHT_IN2 8

#define STEER_SPEED 120
#define MOTOR_SPEED 100

//시간측정용 변수
#define STEER_SPEED_TIME 500
#define STEER_SPEED_TIME_FAST 500
#define MOTOR_SPEED_TIME 100


#define POTENTIOMETER_PIN A5
#define STEER_CENTER_VAL 120    // 왼쪽 85, 오른쪽 66 기준 평균값 좌 134 우 100??
#define STEER_THRESHOLD 1     // 오차 허용 범위

String inputString = "";

void setup() {
  Serial.begin(9600);

  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  stopAll();        // 모든 모터 정지
  delay(100);       // 안정화 대기 (전원 공급 직후)
  steerCenter();    // 조향 중앙 정렬
}

void loop() {
//  int val = potentiometer_Read(POTENTIOMETER_PIN);
//  Serial.println(val);
//  delay(100);  // 너무 빠르지 않게 0.1초마다 출력
  
  if (Serial.available()) {
    char cmd = Serial.read();  // 단일 문자 즉시 읽기

    if (cmd == 'F') {
      moveForward();
    } else if (cmd == 'B') {
      moveBackward();
    } else if (cmd == 'L') {
      steerLeft();
    } else if (cmd == 'R') {
      steerRight();
    } else if (cmd == 'S') {
      stopAll();
    } else if (cmd == 'A') {
      steerFullLeft();
    } else if (cmd == 'D') {
      steerFullRight();
    } else if (cmd == 'C') {
      steerCenter();
    } else if (cmd == 'O') {
      moveForwardSlow();
    } else if (cmd == 'V') {
      moveForwardFast();
    } else if (cmd == 'P') {
      moveBackwardSlow();
    } else if (cmd == 'N') {
      moveBackwardFast();
    } else if (cmd == 'T') {
      String angle_str = Serial.readStringUntil('\n');
      float angle = angle_str.toFloat();
    
      int target_val = angleToPotVal(angle);
      steerTo(target_val);
    }
  }
}

void moveForward() {
  motor_forward(LEFT_IN1, LEFT_IN2, MOTOR_SPEED_TIME);
  motor_forward(RIGHT_IN1, RIGHT_IN2, MOTOR_SPEED_TIME);
}

void moveBackward() {
  motor_backward(LEFT_IN1, LEFT_IN2, MOTOR_SPEED);
  motor_backward(RIGHT_IN1, RIGHT_IN2, MOTOR_SPEED);
}

//시간 측정용 함수
void steerLeft() {
  motor_forward(STEER_IN1, STEER_IN2, STEER_SPEED_TIME_FAST);
  delay(800);  // 100ms 동안만 회전 (즉, 살짝 조향)
  motor_hold(STEER_IN1, STEER_IN2);  // 조향 모터 정지
}

void steerRight() {
  motor_forward(STEER_IN2, STEER_IN1, STEER_SPEED_TIME_FAST);
  delay(800);  // 100ms 동안만 회전
  motor_hold(STEER_IN1, STEER_IN2);  // 정지
}

void steerTo(int target_pot_val) {
  int current_val = potentiometer_Read(POTENTIOMETER_PIN);
  int error = target_pot_val - current_val;

  if (abs(error) <= STEER_THRESHOLD) return;

  if (error > 0) {
    motor_forward(STEER_IN1, STEER_IN2, STEER_SPEED_TIME);
  } else {
    motor_backward(STEER_IN1, STEER_IN2, STEER_SPEED_TIME);
  }

  delay(20);
  motor_hold(STEER_IN1, STEER_IN2);
}

int angleToPotVal(float angle_deg) {
  const float CENTER_DEG = 0.0;
  const int CENTER_VAL = STEER_CENTER_VAL;   // 포텐셔미터 센터 값
  const int LEFT_VAL = 133;     // +20도일 때
  const int RIGHT_VAL = 102;    // -20도일 때

  float scale = (LEFT_VAL - RIGHT_VAL) / 40.0;  // 40도 범위
  return int(CENTER_VAL + angle_deg * scale);
}
////

void stopAll() {
  motor_hold(LEFT_IN1, LEFT_IN2);
  motor_hold(RIGHT_IN1, RIGHT_IN2);
  motor_hold(STEER_IN1, STEER_IN2);
}

////////// 주차함수 ///////////

void steerFullLeft() {
  motor_forward(STEER_IN1, STEER_IN2, 150);
  delay(1000); 
  motor_hold(STEER_IN1, STEER_IN2);
}

void steerFullRight() {
  motor_backward(STEER_IN1, STEER_IN2, 150);
  delay(1000);
  motor_hold(STEER_IN1, STEER_IN2);
}

void steerCenter() {
  int tryCount = 0;

  int val = potentiometer_Read(POTENTIOMETER_PIN);
  int error = val - STEER_CENTER_VAL;
  int target = STEER_CENTER_VAL;

  while (tryCount < 200) {
    val = potentiometer_Read(POTENTIOMETER_PIN);
    error = val - target;

    if (error == 0) break;

    if (error > 0) {
      motor_backward(STEER_IN1, STEER_IN2, 100);
    } else {
      motor_forward(STEER_IN1, STEER_IN2, 100);
    }

    delay(30);
    motor_hold(STEER_IN1, STEER_IN2);
    delay(15);
    Serial.print("count: ");
    Serial.print(tryCount);
    Serial.print("val: ");
    Serial.print(val);
    Serial.print("  target: ");
    Serial.print(target);
    Serial.print("  error: ");
    Serial.println(error);
    tryCount++;
  }

  motor_hold(STEER_IN1, STEER_IN2);
}

void moveForwardFast() {
  motor_forward(LEFT_IN1, LEFT_IN2, 150);
  motor_forward(RIGHT_IN1, RIGHT_IN2, 150);
}

void moveForwardSlow() {
  motor_forward(LEFT_IN1, LEFT_IN2, 60);
  motor_forward(RIGHT_IN1, RIGHT_IN2, 60);
}

void moveBackwardSlow() {
  motor_backward(LEFT_IN1, LEFT_IN2, 100);
  motor_backward(RIGHT_IN1, RIGHT_IN2, 100);
}

void moveBackwardFast() {
  motor_backward(LEFT_IN1, LEFT_IN2, 150);
  motor_backward(RIGHT_IN1, RIGHT_IN2, 150);
}
