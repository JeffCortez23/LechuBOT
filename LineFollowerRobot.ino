#include "config.h"
#include "MotorControl.h"
#include "RobotLogic.h"

void setup() {
  Serial.begin(115200);  // Aumentado a 115200 bauds
  
  setupMotors();
  setupSensors();
  
  // Nueva calibración automática
  calibrateSensors();
  
  Serial.println("✅ Robot listo!");
  delay(1000);
}

void loop() {
  long distance = checkObstacle();
  
  if (distance > 0 && distance < obstacleThreshold) {
    avoidObstacle();
  } else {
    followLine();
  }
  
  delay(10);
}