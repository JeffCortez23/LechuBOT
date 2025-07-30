#include <Arduino.h>
#include "config.h"
#include "RobotLogic.h"
#include "MotorControl.h"
#include "KalmanFilter.h"

// Variables PID
float error = 0;
float lastError = 0;
float integral = 0;
unsigned long lastLineTime = 0;

// Kalman
KalmanFilter pidKalmanFilter(KALMAN_Q_PID, KALMAN_R_PID);
KalmanFilter ultrasonicKalmanFilter(KALMAN_Q_US, KALMAN_R_US);

// Umbrales dinámicos (iniciales)
int BLACK_THRESHOLD_L = 900;
int BLACK_THRESHOLD_C = 900;
int BLACK_THRESHOLD_R = 900;
int WHITE_THRESHOLD_L = 100;
int WHITE_THRESHOLD_C = 100;
int WHITE_THRESHOLD_R = 100;

// Suavizado de sensores
float smoothedL = 0;
float smoothedC = 0;
float smoothedR = 0;
const float SMOOTHING_FACTOR = 0.3; // 30% nuevo valor, 70% histórico

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void calibrateSensors() {
  Serial.println("=== CALIBRACION DE SENSORES ===");
  
  // Calibrar blanco
  Serial.println("Coloca sobre superficie BLANCA y presiona una tecla");
  while(!Serial.available());
  WHITE_THRESHOLD_L = analogRead(SENSOR_L) + 50;
  WHITE_THRESHOLD_C = analogRead(SENSOR_C) + 50;
  WHITE_THRESHOLD_R = analogRead(SENSOR_R) + 50;
  Serial.read();
  
  // Calibrar negro
  Serial.println("Coloca sobre linea NEGRA y presiona una tecla");
  while(!Serial.available());
  BLACK_THRESHOLD_L = analogRead(SENSOR_L) - 50;
  BLACK_THRESHOLD_C = analogRead(SENSOR_C) - 50;
  BLACK_THRESHOLD_R = analogRead(SENSOR_R) - 50;
  Serial.read();
  
  Serial.println("Calibracion completada!");
  Serial.print("Umbral L: "); Serial.println((BLACK_THRESHOLD_L + WHITE_THRESHOLD_L)/2);
  Serial.print("Umbral C: "); Serial.println((BLACK_THRESHOLD_C + WHITE_THRESHOLD_C)/2);
  Serial.print("Umbral R: "); Serial.println((BLACK_THRESHOLD_R + WHITE_THRESHOLD_R)/2);
}

void followLine() {
  // Lectura suavizada
  smoothedL = (SMOOTHING_FACTOR * analogRead(SENSOR_L)) + ((1-SMOOTHING_FACTOR) * smoothedL);
  smoothedC = (SMOOTHING_FACTOR * analogRead(SENSOR_C)) + ((1-SMOOTHING_FACTOR) * smoothedC);
  smoothedR = (SMOOTHING_FACTOR * analogRead(SENSOR_R)) + ((1-SMOOTHING_FACTOR) * smoothedR);

  // Umbral dinámico por sensor
  int thresholdL = (BLACK_THRESHOLD_L + WHITE_THRESHOLD_L) / 2;
  int thresholdC = (BLACK_THRESHOLD_C + WHITE_THRESHOLD_C) / 2;
  int thresholdR = (BLACK_THRESHOLD_R + WHITE_THRESHOLD_R) / 2;
  
  int sL = smoothedL > thresholdL ? 1 : 0;
  int sC = smoothedC > thresholdC ? 1 : 0;
  int sR = smoothedR > thresholdR ? 1 : 0;

  // Debug
  if(DEBUG_MODE) {
    Serial.print("L:"); Serial.print(smoothedL); Serial.print("("); Serial.print(sL);
    Serial.print(") C:"); Serial.print(smoothedC); Serial.print("("); Serial.print(sC);
    Serial.print(") R:"); Serial.print(smoothedR); Serial.print("("); Serial.print(sR); Serial.print(")");
  }

  // Lógica mejorada para curvas
  if (sC) {
    if (!sL && !sR) error = 0;       // Centro perfecto
    else if (sL && !sR) error = -2;  // Leve izquierda
    else if (!sL && sR) error = 2;   // Leve derecha
    else error = 0;                  // Todos activos
    lastLineTime = millis();
  } 
  else if (sL && !sR) {
    error = -4;                      // Curva cerrada izquierda
    lastLineTime = millis();
  } 
  else if (!sL && sR) {
    error = 4;                       // Curva cerrada derecha
    lastLineTime = millis();
  }
  else {
    // No detecta línea
    unsigned long timeSinceLine = millis() - lastLineTime;
    if (timeSinceLine < 200) {
      // Mantener última corrección
      error = (lastError > 0) ? 5 : -5;
    } else {
      // Buscar línea girando
      error = (lastError > 0) ? 8 : -8;
    }
  }

  // Velocidad adaptativa
  int currentBaseSpeed;
  if (abs(error) >= 4) currentBaseSpeed = 100;  // Curva cerrada
  else if (abs(error) >= 2) currentBaseSpeed = 130; // Curva suave
  else currentBaseSpeed = 160;  // Recta

  // PID mejorado
  float filteredError = pidKalmanFilter.update(error);
  integral = constrain(integral + filteredError, -20, 20);
  if (error == 0) integral = 0;
  
  float derivative = filteredError - lastError;
  float correction = (Kp * filteredError) + (Ki * integral) + (Kd * derivative);
  lastError = filteredError;
  
  // Limitar corrección
  correction = constrain(correction, -100, 100);
  
  // Aplicar a motores
  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;
  
  // Compensación para curvas cerradas
  if (abs(error) >= 4) {
    if (correction > 0) { // Giro derecha
      leftSpeed = currentBaseSpeed + abs(correction);
      rightSpeed = currentBaseSpeed / 2;
    } else { // Giro izquierda
      leftSpeed = currentBaseSpeed / 2;
      rightSpeed = currentBaseSpeed + abs(correction);
    }
  }

  // Asegurar límites
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Debug
  if(DEBUG_MODE) {
    Serial.print(" Err:"); Serial.print(error);
    Serial.print(" Spd:"); Serial.print(currentBaseSpeed);
    Serial.print(" L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.println(rightSpeed);
  }

  // Mover motores (todas las ruedas del mismo lado a misma velocidad)
  moveMotors(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
}


void avoidObstacle() {
  Serial.println("¡OBSTÁCULO DETECTADO!");
  stopMotors();
  delay(300);
  
  // Retroceder
  moveMotors(-150, -150, -150, -150);
  delay(500);
  stopMotors();
  delay(300);
  
  // Girar a la izquierda
  moveMotors(-180, -180, 180, 180);
  delay(800);
  stopMotors();
  delay(300);
  
  // Avanzar para rodear
  moveMotors(180, 180, 180, 180);
  delay(1000);
  stopMotors();
  delay(300);
  
  // Girar a la derecha para volver a la línea
  moveMotors(180, 180, -180, -180);
  delay(800);
  stopMotors();
  delay(300);
  
  // Avanzar hasta encontrar la línea
  moveMotors(150, 150, 150, 150);
  delay(1200);
  stopMotors();
  delay(300);
  
  // Girar a la derecha para alinearse
  moveMotors(180, 180, -180, -180);
  delay(600);
  stopMotors();
  delay(500);
}

long checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long raw_distance = pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;

  if (raw_distance > 0 && raw_distance < 400) { // Filtrar lecturas inválidas
    long filtered_distance = ultrasonicKalmanFilter.update(raw_distance);
    return filtered_distance;
  }
  
  return 500; // Sin obstáculo
}