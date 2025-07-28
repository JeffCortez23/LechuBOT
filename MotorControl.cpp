#include <Arduino.h>
#include "config.h"
#include "MotorControl.h"

void setupMotors() {
  pinMode(MOTOR_L_F_FWD, OUTPUT);
  pinMode(MOTOR_L_F_REV, OUTPUT);
  pinMode(MOTOR_L_R_FWD, OUTPUT);
  pinMode(MOTOR_L_R_REV, OUTPUT);
  pinMode(MOTOR_R_F_FWD, OUTPUT);
  pinMode(MOTOR_R_F_REV, OUTPUT);
  pinMode(MOTOR_R_R_FWD, OUTPUT);
  pinMode(MOTOR_R_R_REV, OUTPUT);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  // --- APLICAMOS LA COMPENSACIÓN AQUÍ ---
  int adjustedLeftSpeed = leftSpeed * LEFT_MOTOR_TRIM;

  // Izquierda
  if (adjustedLeftSpeed >= 0) {
    analogWrite(MOTOR_L_F_FWD, adjustedLeftSpeed);
    analogWrite(MOTOR_L_R_FWD, adjustedLeftSpeed);
    digitalWrite(MOTOR_L_F_REV, LOW);
    digitalWrite(MOTOR_L_R_REV, LOW);
  } else {
    analogWrite(MOTOR_L_F_REV, -adjustedLeftSpeed);
    analogWrite(MOTOR_L_R_REV, -adjustedLeftSpeed);
    digitalWrite(MOTOR_L_F_FWD, LOW);
    digitalWrite(MOTOR_L_R_FWD, LOW);
  }
  
  // Derecha (sin cambios)
  if (rightSpeed >= 0) {
    analogWrite(MOTOR_R_F_FWD, rightSpeed);
    analogWrite(MOTOR_R_R_FWD, rightSpeed);
    digitalWrite(MOTOR_R_F_REV, LOW);
    digitalWrite(MOTOR_R_R_REV, LOW);
  } else {
    analogWrite(MOTOR_R_F_REV, -rightSpeed);
    analogWrite(MOTOR_R_R_REV, -rightSpeed);
    digitalWrite(MOTOR_R_F_FWD, LOW);
    digitalWrite(MOTOR_R_F_FWD, LOW);
  }
}

void stopMotors() {
  moveMotors(0, 0);
}