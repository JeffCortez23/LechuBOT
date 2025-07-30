#include <Arduino.h>
#include "config.h"
#include "RobotLogic.h"
#include "MotorControl.h"

// Variables de control
float error = 0;
float lastError = 0;
unsigned long lastLineTime = 0;

// Suavizado de sensores
float smoothedL = 0;
float smoothedC = 0;
float smoothedR = 0;
const float SMOOTHING_FACTOR = 0.3;

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void followLine() {
  // Lectura suavizada de sensores
  smoothedL = (SMOOTHING_FACTOR * analogRead(SENSOR_L)) + ((1-SMOOTHING_FACTOR) * smoothedL);
  smoothedC = (SMOOTHING_FACTOR * analogRead(SENSOR_C)) + ((1-SMOOTHING_FACTOR) * smoothedC);
  smoothedR = (SMOOTHING_FACTOR * analogRead(SENSOR_R)) + ((1-SMOOTHING_FACTOR) * smoothedR);

  const int FIXED_LINE_THRESHOLD = 500;
  
  int sL = smoothedL > FIXED_LINE_THRESHOLD ? 1 : 0;
  int sC = smoothedC > FIXED_LINE_THRESHOLD ? 1 : 0;
  int sR = smoothedR > FIXED_LINE_THRESHOLD ? 1 : 0;

  if(DEBUG_MODE) {
    Serial.print("L:"); Serial.print(smoothedL); Serial.print("("); Serial.print(sL);
    Serial.print(") C:"); Serial.print(smoothedC); Serial.print("("); Serial.print(sC);
    Serial.print(") R:"); Serial.print(smoothedR); Serial.print("("); Serial.print(sR); Serial.print(")");
  }

  // Lógica de error
  if (sC) {
    if (!sL && !sR) error = 0;
    else if (sL && !sR) error = -2;
    else if (!sL && sR) error = 2;
    else error = 0;
    lastLineTime = millis();
  } 
  else if (sL && !sR) {
    error = -4;
    lastLineTime = millis();
  } 
  else if (!sL && sR) {
    error = 4;
    lastLineTime = millis();
  }
  else {
    unsigned long timeSinceLine = millis() - lastLineTime;
    if (timeSinceLine < 200) {
      error = (lastError > 0) ? 5 : -5;
    } else {
      error = (lastError > 0) ? 8 : -8;
    }
  }

  // Velocidad adaptativa
  int currentBaseSpeed;
  if (abs(error) >= 4) {
    currentBaseSpeed = 90;
  } else if (abs(error) >= 2) {
    currentBaseSpeed = 110;
  } else {
    currentBaseSpeed = 140;
  }

  // --- CONTROLADOR PD (SIN COMPONENTE INTEGRAL) ---
  float derivative = error - lastError;
  float correction = (Kp * error) + (Kd * derivative); // <-- ¡Solo P y D!
  lastError = error;
  
  correction = constrain(correction, -100, 100);
  
  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;
  
  if (abs(error) >= 4) {
    if (correction > 0) {
      leftSpeed = currentBaseSpeed + abs(correction);
      rightSpeed = currentBaseSpeed / 2;
    } else {
      leftSpeed = currentBaseSpeed / 2;
      rightSpeed = currentBaseSpeed + abs(correction);
    }
  }

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  if(DEBUG_MODE) {
    Serial.print(" Err:"); Serial.print(error);
    Serial.print(" Spd:"); Serial.print(currentBaseSpeed);
    Serial.print(" L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.println(rightSpeed);
  }

  moveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
}

void avoidObstacle() {
  Serial.println("¡OBSTÁCULO! Maniobra TÁCTICA iniciada.");
  stopMotors();
  delay(250);
  
  Serial.println("Paso 1: Creando espacio...");
  moveMotors(-150, -150, -150, -150);
  delay(600);
  stopMotors();
  delay(250);
  
  Serial.println("Paso 2: Apuntando para rodear...");
  moveMotors(-180, -180, 180, 180);
  delay(600);
  stopMotors();
  delay(250);
  
  Serial.println("Paso 3: Pasando el obstáculo...");
  moveMotors(150, 150, 150, 150);
  delay(1200);
  stopMotors();
  delay(250);
  
  Serial.println("Paso 4: Encarando la línea...");
  moveMotors(180, 180, -180, -180);
  delay(600);
  stopMotors();
  delay(250);

  Serial.println("Paso 5: BÚSQUEDA ACTIVA de la línea...");
  moveMotors(120, 120, 120, 120);
  
  unsigned long searchStartTime = millis();
  while(analogRead(SENSOR_R) < 500) { 
    if (millis() - searchStartTime > 2500) {
      Serial.println("¡Línea no encontrada! Abortando.");
      break;
    }
    delay(10);
  }
  
  Serial.println("Paso 6: REALINEACIÓN sobre la línea...");
  moveMotors(-100, -100, 100, 100);
  delay(200);
  stopMotors();
  
  Serial.println("Maniobra completada. Seguimiento reanudado.");
  lastError = 0;
}

long checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long raw_distance = pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;

  if (raw_distance > 0 && raw_distance < 400) {
    return raw_distance;
  }
  
  return 500;
}