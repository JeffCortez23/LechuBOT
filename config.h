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

// --- CONSTANTES DE CONTROL (PD Balanceado) ---
const float Kp = 15.0; // Aumentado para una reacción más fuerte al error.
const float Kd = 20.0; // Aumentado para mayor estabilidad en la corrección.
const int obstacleThreshold = 20;

// Compensación motores
const float LEFT_MOTOR_TRIM = 1.05;

#define DEBUG_MODE true

#endif