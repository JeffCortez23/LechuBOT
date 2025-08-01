#include "config.h"
#include "MotorControl.h"
#include "RobotLogic.h"

unsigned long startTime;

void setup() {
  Serial.begin(9600);
  
  setupMotors();
  setupSensors();
  
  Serial.println("✅ Robot listo!");
  startTime = millis();
}

void loop() {
  if (millis() - startTime < 1500) {
    followLine();
  } else {
    long distance = checkObstacle();
    if (distance > 0 && distance < obstacleThreshold) {
      avoidObstacle();
    } else {
      followLine();
    }
  }
  
  delay(10);
}