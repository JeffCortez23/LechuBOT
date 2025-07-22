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
  Serial.print("Dist: ");
  Serial.print(distance);
  Serial.print(" cm | ");
  
  if (distance < obstacleThreshold && distance > 0) {
    avoidObstacle();
  } else {
    followLine();
  }
  
  delay(50); 
}