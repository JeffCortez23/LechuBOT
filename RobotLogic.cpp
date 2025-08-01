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

  // Lógica de error (sin cambios)
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

  // Velocidad adaptativa (sin cambios)
  int currentBaseSpeed;
  if (abs(error) >= 4) {
    currentBaseSpeed = 90;
  } else if (abs(error) >= 2) {
    currentBaseSpeed = 110;
  } else {
    currentBaseSpeed = 140;
  }

  // --- CONTROLADOR PD CON CORRECCIONES ---
  float derivative = error - lastError;
  float correction = (Kp * error) + (Kd * derivative);
  lastError = error;
  
  // ¡NUEVO! Limitamos la corrección para evitar reacciones exageradas.
  correction = constrain(correction, -150, 150);
  
  int leftSpeed = currentBaseSpeed + correction;
  int rightSpeed = currentBaseSpeed - correction;
  
  // --- LÓGICA DE GIRO CORREGIDA (Point Turn) ---
  if (abs(error) >= 4) {
    if (error > 0) { // Desviado a la izquierda, necesita girar a la DERECHA.
      leftSpeed = 200;   // Rueda exterior (izquierda) gira más rápido.
      rightSpeed = 0;    // Rueda interior (derecha) se detiene para un giro cerrado.
    } else {           // Desviado a la derecha, necesita girar a la IZQUIERDA.
      leftSpeed = 0;     // Rueda interior (izquierda) se detiene.
      rightSpeed = 200;  // Rueda exterior (derecha) gira más rápido.
    }
  }

  // Nos aseguramos de que las velocidades no excedan el máximo de PWM (0-255).
  // La función moveMotors ya maneja velocidades negativas, pero es bueno mantener esto por seguridad.
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

  // --- PASO 5 MEJORADO: BÚSQUEDA ACTIVA DE LÍNEA ---
  Serial.println("Paso 5: BÚSQUEDA ACTIVA de la línea...");
  moveMotors(130, 130, -130, -130); // Gira sobre su eje hacia la derecha para buscar.
  
  unsigned long searchStartTime = millis();
  // Espera a que CUALQUIER sensor (central, derecho o izquierdo) encuentre la línea
  while(analogRead(SENSOR_C) < 500 && analogRead(SENSOR_R) < 500 && analogRead(SENSOR_L) < 500) { 
    if (millis() - searchStartTime > 4000) { // Aumenta el tiempo de búsqueda por si acaso
      Serial.println("¡Línea no encontrada! Abortando maniobra.");
      // Opcional: Implementar una estrategia de emergencia, como detenerse.
      stopMotors(); 
      return; // Salir de la función de evasión
    }
    delay(10);
  }
  
  // --- PASO 6: REALINEACIÓN ---
  Serial.println("¡Línea encontrada! Realineando...");
  stopMotors();
  delay(100);
  
  Serial.println("Maniobra completada. Seguimiento reanudado.");
  lastError = 0; // Reinicia el error para un arranque suave
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