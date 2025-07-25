#include <Arduino.h>
#include "config.h"
#include "RobotLogic.h"
#include "MotorControl.h"

// Variables Globales para el PID
float error = 0;
float lastError = 0;
float integral = 0;

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void followLine() {
  // --- Calibración ---
  int threshold = 535; // Tu valor umbral ya calculado

  // --- Lectura de Sensores ---
  int sL = analogRead(SENSOR_L) > threshold ? 1 : 0;
  int sC = analogRead(SENSOR_C) > threshold ? 1 : 0;
  int sR = analogRead(SENSOR_R) > threshold ? 1 : 0;

  Serial.print("Sensores[L,C,R]: ");
  Serial.print(sL); Serial.print(","); Serial.print(sC); Serial.print(","); Serial.print(sR);
  Serial.print(" | ");

  // --- Lógica de Decisión para Anchos Variables ---

  // CASO 1: IR RECTO
  // El robot debe ir recto si está perfectamente centrado (0 1 0) o si está
  // completamente dentro de la línea ancha (1 1 1).
  if ((sL == 0 && sC == 1 && sR == 0) || (sL == 1 && sC == 1 && sR == 1)) {
    error = 0;
    integral = 0; // Resetea el integral, no hay error acumulado.
    Serial.print("Accion: ✅ Recto | ");
  }
  
  // CASO 2: CORRECCIONES SUAVES (sobre la línea ancha)
  else if (sL == 1 && sC == 1 && sR == 0) { // Un poco a la derecha, corregir a la izquierda
    error = -1;
    Serial.print("Accion: ⬅️ Corrigiendo | ");
  }
  else if (sL == 0 && sC == 1 && sR == 1) { // Un poco a la izquierda, corregir a la derecha
    error = 1;
    Serial.print("Accion: ➡️ Corrigiendo | ");
  }

  // CASO 3: CORRECCIONES FUERTES (saliendo de la línea)
  else if (sL == 1 && sC == 0 && sR == 0) { // Muy a la derecha
    error = -3;
    Serial.print("Accion: ⬅️⬅️ Giro Fuerte | ");
  }
  else if (sL == 0 && sC == 0 && sR == 1) { // Muy a la izquierda
    error = 3;
    Serial.print("Accion: ➡️➡️ Giro Fuerte | ");
  }

  // CASO 4: LÍNEA PERDIDA
  else if (sL == 0 && sC == 0 && sR == 0) {
    // Gira en la última dirección del error para buscar la línea
    if (lastError > 0) { error = 5; } else { error = -5; }
    Serial.print("Accion: ❓ Buscando... | ");
  }

  // --- Cálculo y Aplicación del PID ---
  integral += error;
  float correction = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));
  lastError = error;
  
  int leftSpeed = constrain(baseSpeed + correction, -255, 255);
  int rightSpeed = constrain(baseSpeed - correction, -255, 255);

  moveMotors(leftSpeed, rightSpeed);
}

// ... (resto de tu código)

void avoidObstacle() {
  Serial.println("--- 🚧 ¡OBSTACULO DETECTADO! INICIANDO MANIOBRA AMPLIA 🚧 ---");
  
  // 1. DETENERSE Y RETROCEDER
  // Creamos espacio para poder girar cómodamente.
  Serial.println("Paso 1: Creando espacio...");
  stopMotors();
  delay(300);
  moveMotors(-150, -150); // Marcha atrás suave
  delay(400);            // Retrocede por 0.4 segundos
  stopMotors();
  delay(300);

  // 2. GIRO DE 90 GRADOS (hacia la izquierda)
  // Apuntamos hacia un lado para empezar a rodear.
  // ¡AJUSTA EL DELAY HASTA QUE EL GIRO SEA DE 90 GRADOS EXACTOS!
  Serial.println("Paso 2: Girando 90 grados...");
  moveMotors(-200, 200); // Gira sobre su eje (Izquierda atrás, Derecha adelante)
  delay(750);            // ¡ESTE VALOR ES CRÍTICO! Ajústalo para tu robot.
  stopMotors();
  delay(300);

  // 3. AVANZAR EN LÍNEA RECTA (para pasar el obstáculo por el lado)
  // Este es el tramo que define qué tan lejos rodeamos el objeto.
  Serial.println("Paso 3: Avanzando por el lateral...");
  moveMotors(200, 200);
  delay(1200);           // ¡Aumenta este valor si chocas por el lado!
  stopMotors();
  delay(300);

  // 4. SEGUNDO GIRO DE 90 GRADOS (hacia la derecha)
  // Volvemos a apuntar en la dirección original de la pista.
  Serial.println("Paso 4: Re-orientando...");
  moveMotors(200, -200); // Gira en sentido contrario
  delay(750);            // Usa el MISMO valor que en el paso 2.
  stopMotors();
  delay(300);

  // 5. AVANZAR PARA SOBREPASAR COMPLETAMENTE
  // Nos aseguramos de que la parte trasera del robot también haya pasado el obstáculo.
  Serial.println("Paso 5: Asegurando el paso...");
  moveMotors(200, 200);
  delay(1500);           // ¡Aumenta este valor si el robot gira y choca con el obstáculo!
  stopMotors();
  delay(300);

  // 6. TERCER GIRO DE 90 GRADOS (hacia la derecha)
  // Giramos para encarar de nuevo la pista y buscar la línea.
  Serial.println("Paso 6: Buscando la línea...");
  moveMotors(200, -200);
  delay(750);            // Usa el MISMO valor que en el paso 2.
  stopMotors();
  delay(300);
  
  // 7. FIN DE LA MANIOBRA
  // El robot ahora avanzará y la función followLine() se encargará de re-alinearse
  // cuando uno de los sensores encuentre la línea negra.
  Serial.println("--- ✅ FIN DE MANIOBRA. Reanudando seguimiento. ---");
}

// ... (El resto de tus funciones como followLine y checkObstacle)
long checkObstacle() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(ECHO_PIN, HIGH, 30000) * 0.034 / 2;
}