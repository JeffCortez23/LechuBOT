#include <Arduino.h>
#include "config.h"
#include "RobotLogic.h"
#include "MotorControl.h"
#include "KalmanFilter.h" 

// Variables Globales para el PID
float error = 0;
float lastError = 0;
float integral = 0;

// DOS INSTANCIAS DEL FILTRO DE KALMAN
KalmanFilter pidKalmanFilter(KALMAN_Q_PID, KALMAN_R_PID);
KalmanFilter ultrasonicKalmanFilter(KALMAN_Q_US, KALMAN_R_US);

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void followLine() {
  int threshold = 525;
  int sL = analogRead(SENSOR_L) > threshold ? 1 : 0;
  int sC = analogRead(SENSOR_C) > threshold ? 1 : 0;
  int sR = analogRead(SENSOR_R) > threshold ? 1 : 0;

  if ((sL == 0 && sC == 1 && sR == 0) || (sL == 1 && sC == 1 && sR == 1) || (sL == 1 && sC == 0 && sR == 1)) {
    error = 0;
  } else if (sL == 1 && sC == 1 && sR == 0) {
    error = -1;
  } else if (sL == 0 && sC == 1 && sR == 1) {
    error = 1;
  } else if (sL == 1 && sC == 0 && sR == 0) {
    error = -3;
  } else if (sL == 0 && sC == 0 && sR == 1) {
    error = 3;
  } else if (sL == 0 && sC == 0 && sR == 0) {
    if (lastError > 0) { error = 5; } else { error = -5; }
  }

  float filteredError = pidKalmanFilter.update(error);

  integral += filteredError;
  if (error == 0) { integral = 0; }
  float correction = (Kp * filteredError) + (Ki * integral) + (Kd * (filteredError - lastError));
  lastError = filteredError;
  
  int leftBaseSpeed = baseSpeed + correction;
  int rightBaseSpeed = baseSpeed - correction;
  int frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;

  if (correction > 0) {
    frontRightSpeed = rightBaseSpeed * 0.8;
    rearRightSpeed = rightBaseSpeed;
    frontLeftSpeed = leftBaseSpeed;
    rearLeftSpeed = leftBaseSpeed;
  } else if (correction < 0) {
    frontLeftSpeed = leftBaseSpeed * 0.8;
    rearLeftSpeed = leftBaseSpeed;
    frontRightSpeed = rightBaseSpeed;
    rearRightSpeed = rightBaseSpeed;
  } else {
    frontLeftSpeed = baseSpeed;
    rearLeftSpeed = baseSpeed;
    frontRightSpeed = baseSpeed;
    rearRightSpeed = baseSpeed;
  }

  moveMotors(
    constrain(frontLeftSpeed, -255, 255),
    constrain(rearLeftSpeed, -255, 255),
    constrain(frontRightSpeed, -255, 255),
    constrain(rearRightSpeed, -255, 255)
  );
}

void avoidObstacle() {
  stopMotors();
  delay(300);
  moveMotors(-150, -150, -150, -150);
  delay(400);
  stopMotors();
  delay(300);
  moveMotors(-200, -200, 200, 200);
  delay(750);
  stopMotors();
  delay(300);
  moveMotors(200, 200, 200, 200);
  delay(1200);
  stopMotors();
  delay(300);
  moveMotors(200, 200, -200, -200);
  delay(750);
  stopMotors();
  delay(300);
  moveMotors(200, 200, 200, 200);
  delay(1500);
  stopMotors();
  delay(300);
  moveMotors(200, 200, -200, -200);
  delay(750);
  stopMotors();
  delay(300);
}

long checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long raw_distance = pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;

  if (raw_distance > 0) {
    long filtered_distance = ultrasonicKalmanFilter.update(raw_distance);
    return filtered_distance;
  }
  
  return 500;
}