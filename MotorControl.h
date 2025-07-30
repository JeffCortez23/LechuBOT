#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void setupMotors();
void moveMotors(int frontLeftSpeed, int rearLeftSpeed, int frontRightSpeed, int rearRightSpeed);
void stopMotors();

#endif