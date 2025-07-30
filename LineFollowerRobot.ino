#include "config.h"
#include "MotorControl.h"
#include "RobotLogic.h"

void setup() {
  Serial.begin(9600);
  
  setupMotors();
  setupSensors();

  Serial.println("✅ Sistema de Robot Modularizado: INICIADO");
  delay(1000);
}

void loop() {
  long distance = checkObstacle();

  if (distance < obstacleThreshold && distance > 0) {
    Serial.println("Distancia Ultrasónico: ");
    Serial.println(distance);
    Serial.println(" cm. ¡Obstáculo detectado!");
    avoidObstacle();
  } else {
    followLine();
  }
  
  delay(10); 
}