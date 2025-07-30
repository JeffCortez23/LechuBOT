#ifndef ROBOT_LOGIC_H
#define ROBOT_LOGIC_H

void setupSensors();
void followLine();
void avoidObstacle();
long checkObstacle();
void calibrateSensors();  // Nueva función de calibración

// Variables para calibración
extern int BLACK_THRESHOLD_L;
extern int BLACK_THRESHOLD_C;
extern int BLACK_THRESHOLD_R;
extern int WHITE_THRESHOLD_L;
extern int WHITE_THRESHOLD_C;
extern int WHITE_THRESHOLD_R;

#endif