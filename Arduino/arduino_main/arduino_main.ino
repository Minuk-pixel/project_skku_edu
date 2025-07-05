#include <Car_Library.h>

#define STEER_IN1 12
#define STEER_IN2 13
#define LEFT_IN1 11
#define LEFT_IN2 10
#define RIGHT_IN1 8
#define RIGHT_IN2 9
#define MOTOR_SPEED 30
#define STEER_SPEED 170

String inputString = "";

void setup() {
  Serial.begin(9600);

  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  stopAll();
}

void loop() {
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
    }
  }
}

void moveForward() {
  motor_forward(LEFT_IN1, LEFT_IN2, MOTOR_SPEED);
  motor_forward(RIGHT_IN1, RIGHT_IN2, MOTOR_SPEED);
}

void moveBackward() {
  motor_backward(LEFT_IN1, LEFT_IN2, MOTOR_SPEED);
  motor_backward(RIGHT_IN1, RIGHT_IN2, MOTOR_SPEED);
}

void steerLeft() {
  motor_forward(STEER_IN1, STEER_IN2, STEER_SPEED);
  delay(40);  // 100ms 동안만 회전 (즉, 살짝 조향)
  motor_hold(STEER_IN1, STEER_IN2);  // 조향 모터 정지
}

void steerRight() {
  motor_forward(STEER_IN2, STEER_IN1, STEER_SPEED);
  delay(40);  // 100ms 동안만 회전
  motor_hold(STEER_IN1, STEER_IN2);  // 정지
}

void stopAll() {
  motor_hold(LEFT_IN1, LEFT_IN2);
  motor_hold(RIGHT_IN1, RIGHT_IN2);
  motor_hold(STEER_IN1, STEER_IN2);
}
