#include "config.h"
#include "MotorControl.h"
#include "RobotLogic.h"

unsigned long startTime; // Variable para guardar el tiempo de inicio

void setup() {
  Serial.begin(115200);
  
  setupMotors();
  setupSensors();
  
  Serial.println("✅ Robot listo!");
  startTime = millis(); // Guardamos el momento exacto en que termina la configuración.
}

void loop() {
  // --- SOLUCIÓN DEFINITIVA AL ARRANQUE EN REVERSA ---
  // Durante el primer 1.5 segundo, el robot SÓLO seguirá la línea.
  if (millis() - startTime < 1500) {
    followLine(); // Forzamos el seguimiento de línea, ignorando todo lo demás.
  } else {
    // Después del período de gracia, el robot funciona normalmente.
    long distance = checkObstacle();
    if (distance > 0 && distance < obstacleThreshold) {
      avoidObstacle();
    } else {
      followLine();
    }
  }
  
  delay(10);
}