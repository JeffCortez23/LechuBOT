#include <Arduino.h>
#include "config.h"
#include "RobotLogic.h"
#include "MotorControl.h"
#include "KalmanFilter.h" 

// Variables Globales para el PID
float error = 0;
float lastError = 0;
float integral = 0;

// Instancia del Filtro de Kalman
KalmanFilter kalmanFilter(KALMAN_Q, KALMAN_R);

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void followLine() {
  int threshold = 525; // Umbral calibrado

  int sL = analogRead(SENSOR_L) > threshold ? 1 : 0;
  int sC = analogRead(SENSOR_C) > threshold ? 1 : 0;
  int sR = analogRead(SENSOR_R) > threshold ? 1 : 0;

  Serial.print("Sensores[L,C,R]: ");
  Serial.print(sL); Serial.print(sC); Serial.print(sR);

  // Lógica de cálculo de error
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

  float filteredError = kalmanFilter.update(error);

  integral += filteredError;
  if (error == 0) { integral = 0; }
  float correction = (Kp * filteredError) + (Ki * integral) + (Kd * (filteredError - lastError));
  lastError = filteredError;
  
  Serial.print(" | Err: "); Serial.print(error);
  Serial.print(" | Corr: "); Serial.print(correction, 2);

  int leftBaseSpeed = baseSpeed + correction;
  int rightBaseSpeed = baseSpeed - correction;
  int frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;

  if (correction > 0) { // Girando a la derecha
    frontRightSpeed = rightBaseSpeed * 0.8;
    rearRightSpeed = rightBaseSpeed;
    frontLeftSpeed = leftBaseSpeed;
    rearLeftSpeed = leftBaseSpeed;
  } else if (correction < 0) { // Girando a la izquierda
    frontLeftSpeed = leftBaseSpeed * 0.8;
    rearLeftSpeed = leftBaseSpeed;
    frontRightSpeed = rightBaseSpeed;
    rearRightSpeed = rightBaseSpeed;
  } else { // Yendo recto
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
  Serial.println("\n--- INICIO MANIOBRA DE EVASIÓN ---");
  Serial.println("Paso 1: Deteniendo motores.");
  stopMotors();
  delay(300);
  
  Serial.println("Paso 2: Retrocediendo...");
  moveMotors(-150, -150, -150, -150);
  delay(400);
  
  stopMotors();
  delay(300);
  
  Serial.println("Paso 3: Girando a la izquierda...");
  moveMotors(-200, -200, 200, 200);
  delay(750);
  
  stopMotors();
  delay(300);
  
  Serial.println("Paso 4: Avanzando por el lateral...");
  moveMotors(200, 200, 200, 200);
  delay(1200);
  
  stopMotors();
  delay(300);
  
  Serial.println("Paso 5: Girando a la derecha (re-orientando)...");
  moveMotors(200, 200, -200, -200);
  delay(750);
  
  stopMotors();
  delay(300);
  
  Serial.println("Paso 6: Avanzando para sobrepasar...");
  moveMotors(200, 200, 200, 200);
  delay(1500);
  
  stopMotors();
  delay(300);
  
  Serial.println("Paso 7: Girando a la derecha (buscando línea)...");
  moveMotors(200, 200, -200, -200);
  delay(750);
  
  stopMotors();
  delay(300);
  Serial.println("--- FIN DE MANIOBRA ---");
}

long checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;
}