#include <Arduino.h>
#include "config.h"
#include "RobotLogic.h"
#include "MotorControl.h" // Necesita saber cómo mover los motores

// Variables globales para el control PD
float error = 0;
float lastError = 0;

void setupSensors() {
  pinMode(SENSOR_L, INPUT);
  pinMode(SENSOR_C, INPUT);
  pinMode(SENSOR_R, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void followLine() {
  int sL = !digitalRead(SENSOR_L);
  int sC = !digitalRead(SENSOR_C);
  int sR = !digitalRead(SENSOR_R);

  Serial.print("Sensores[L,C,R]: ");
  Serial.print(sL); Serial.print(","); Serial.print(sC); Serial.print(","); Serial.print(sR);
  Serial.print(" | ");

  if (sL == 1 && sC == 0 && sR == 1) { error = 0; Serial.print("Accion: ✅ Centrado | "); } 
  else if (sL == 1 && sC == 1 && sR == 0) { error = 1; Serial.print("Accion: ➡️ Corrigiendo | "); } 
  else if (sL == 0 && sC == 1 && sR == 1) { error = -1; Serial.print("Accion: ⬅️ Corrigiendo | "); } 
  else if (sL == 1 && sC == 0 && sR == 0) { error = 3; Serial.print("Accion: ➡️➡️ Giro Fuerte | "); } 
  else if (sL == 0 && sC == 0 && sR == 1) { error = -3; Serial.print("Accion: ⬅️⬅️ Giro Fuerte | "); } 
  else if (sL == 1 && sC == 1 && sR == 1) {
    if (lastError > 0) { error = 5; Serial.print("Accion: ❓ Buscando -> | "); } 
    else { error = -5; Serial.print("Accion: ❓ Buscando <- | "); }
  }

  Serial.print("Error: "); Serial.print(error); Serial.print(" | ");
  float correction = (Kp * error) + (Kd * (error - lastError));
  lastError = error;
  Serial.print("Corr: "); Serial.print(correction); Serial.print(" | ");

  int leftSpeed = constrain(baseSpeed - correction, -255, 255);
  int rightSpeed = constrain(baseSpeed + correction, -255, 255);

  Serial.print("Vels[L,R]: ");
  Serial.print(leftSpeed); Serial.print(","); Serial.print(rightSpeed);
  Serial.println();

  moveMotors(leftSpeed, rightSpeed);
}

void avoidObstacle() {
  Serial.println("--- 🚧 ¡OBSTACULO! INICIANDO MANIOBRA DE EVASION 🚧 ---");
  stopMotors();
  delay(300);
  moveMotors(200, -200); delay(400); // Girar
  moveMotors(200, 200); delay(500);  // Avanzar
  // ...y el resto de la maniobra
  stopMotors();
  Serial.println("--- ✅ FIN DE MANIOBRA. ---");
}

long checkObstacle() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;
}