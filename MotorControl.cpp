#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"

void setupMotors() {
  pinMode(MOTOR_FL_FWD, OUTPUT);
  pinMode(MOTOR_FL_REV, OUTPUT);
  pinMode(MOTOR_FR_FWD, OUTPUT);
  pinMode(MOTOR_FR_REV, OUTPUT);
  pinMode(MOTOR_RL_FWD, OUTPUT);
  pinMode(MOTOR_RL_REV, OUTPUT);
  pinMode(MOTOR_RR_FWD, OUTPUT);
  pinMode(MOTOR_RR_REV, OUTPUT);
}

void moveMotors(int frontLeftSpeed, int rearLeftSpeed, int frontRightSpeed, int rearRightSpeed) {
  float adjustedFLSpeed = frontLeftSpeed * LEFT_MOTOR_TRIM;
  if (adjustedFLSpeed >= 0) {
    analogWrite(MOTOR_FL_FWD, adjustedFLSpeed);
    digitalWrite(MOTOR_FL_REV, LOW);
  } else {
    analogWrite(MOTOR_FL_REV, -adjustedFLSpeed);
    digitalWrite(MOTOR_FL_FWD, LOW);
  }

  float adjustedRLSpeed = rearLeftSpeed * LEFT_MOTOR_TRIM;
  if (adjustedRLSpeed >= 0) {
    analogWrite(MOTOR_RL_FWD, adjustedRLSpeed);
    digitalWrite(MOTOR_RL_REV, LOW);
  } else {
    analogWrite(MOTOR_RL_REV, -adjustedRLSpeed);
    digitalWrite(MOTOR_RL_FWD, LOW);
  }

  if (frontRightSpeed >= 0) {
    analogWrite(MOTOR_FR_FWD, frontRightSpeed);
    digitalWrite(MOTOR_FR_REV, LOW);
  } else {
    analogWrite(MOTOR_FR_REV, -frontRightSpeed);
    digitalWrite(MOTOR_FR_FWD, LOW);
  }

  if (rearRightSpeed >= 0) {
    analogWrite(MOTOR_RR_FWD, rearRightSpeed);
    digitalWrite(MOTOR_RR_REV, LOW);
  } else {
    analogWrite(MOTOR_RR_REV, -rearRightSpeed);
    digitalWrite(MOTOR_RR_FWD, LOW);
  }
}

void stopMotors() {
  moveMotors(0, 0, 0, 0);
}