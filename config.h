#ifndef CONFIG_H
#define CONFIG_H

// --- DEFINICIÓN DE PINES ---
#define SENSOR_L A8
#define SENSOR_C A9
#define SENSOR_R A10

// Motor Driver 1 (Delantero)
#define MOTOR_FL_FWD 4
#define MOTOR_FL_REV 5
#define MOTOR_FR_FWD 9
#define MOTOR_FR_REV 8

// Motor Driver 2 (Trasero)
#define MOTOR_RL_FWD 6
#define MOTOR_RL_REV 7
#define MOTOR_RR_FWD 11
#define MOTOR_RR_REV 10

// Ultrasonico
#define TRIG_PIN 14
#define ECHO_PIN 15

// --- CONSTANTES DE CONTROL (OPTIMIZADAS) ---
const float Kp = 14.0;            // Más suave que 18.0
const float Ki = 0.03;            // Reducido para evitar oscilaciones
const float Kd = 10.0;            // Más estable que 15.0
const int obstacleThreshold = 20;

// Umbrales dinámicos (se calibrarán)
extern int BLACK_THRESHOLD_L;
extern int BLACK_THRESHOLD_C;
extern int BLACK_THRESHOLD_R;
extern int WHITE_THRESHOLD_L;
extern int WHITE_THRESHOLD_C;
extern int WHITE_THRESHOLD_R;

// Compensación motores
const float LEFT_MOTOR_TRIM = 1.05;

// Kalman
const float KALMAN_Q_PID = 0.1;
const float KALMAN_R_PID = 4.0;
const float KALMAN_Q_US = 0.1;
const float KALMAN_R_US = 4.0;

#define DEBUG_MODE true

#endif