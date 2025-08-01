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

  float linePosition = 0;
  float totalWeight = 0;

  if (smoothedL > 500) {
    linePosition += -2 * smoothedL;
    totalWeight += smoothedL;
  }
  if (smoothedC > 500) {
    linePosition += 0 * smoothedC;  
    totalWeight += smoothedC;
  }
  if (smoothedR > 500) {
    linePosition += 2 * smoothedR;  
    totalWeight += smoothedR;
  }

  // Calcula el error real si se detecta la línea
  if (totalWeight > 0) {
    error = linePosition / totalWeight;
    lastLineTime = millis();
  } else {
    unsigned long timeSinceLine = millis() - lastLineTime;
    if (timeSinceLine > 200) {
      error = (lastError > 0) ? 8 : -8;
    } else {
      error = (lastError > 0) ? 4 : -4;
    }
  }

  // Velocidad adaptativa 
  int currentBaseSpeed;
  if (abs(error) > 3) { 
    currentBaseSpeed = 90;
  } else if (abs(error) > 1) {
    currentBaseSpeed = 110;
  } else {
    currentBaseSpeed = 110; 
  }
  
  // --- CONTROLADOR PD CON CORRECCIONES ---
  float derivative = error - lastError;
  float correction = (Kp * error) + (Kd * derivative);
  lastError = error;
  
  // Limitamos la corrección para evitar reacciones exageradas.
  correction = constrain(correction, -255, 255);
  
  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;
  
  // Nos aseguramos de que las velocidades no excedan el máximo de PWM (0-255).
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

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
  moveMotors(130, 130, -130, -130);
  
  unsigned long searchStartTime = millis();
  // Espera a que CUALQUIER sensor (central, derecho o izquierdo) encuentre la línea
  while(analogRead(SENSOR_C) < 500 && analogRead(SENSOR_R) < 500 && analogRead(SENSOR_L) < 500) { 
    if (millis() - searchStartTime > 4000) { 
      Serial.println("¡Línea no encontrada! Abortando maniobra.");
      stopMotors(); 
      return; 
    }
    delay(10);
  }

  Serial.println("¡Línea encontrada! Realineando...");
  stopMotors();
  delay(100);
  
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