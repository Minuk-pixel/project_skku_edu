#include "Arduino.h"
#include "Car_Library.h"

float ultrasonic_distance(int trigPin, int echoPin)
{
    long distance, duration;

    digitalWrite(trigPin, LOW);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = ((float)(340 * duration) / 1000) / 2;

    return distance;
}

int potentiometer_Read(int pin)
{
    int value;

    value = analogRead(pin) / 4;

    return value;
}

void motor_forward(int IN1, int IN2, int speed)
{
    analogWrite(IN1, speed);
    analogWrite(IN2, LOW);
}

void motor_backward(int IN1, int IN2, int speed)
{
    analogWrite(IN1, LOW);
    analogWrite(IN2, speed);
}

void motor_hold(int IN1, int IN2)
{
    analogWrite(IN1, LOW);
    analogWrite(IN2, LOW);
}