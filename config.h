#ifndef CONFIG_H
#define CONFIG_H

// --- DEFINICIÓN DE PINES (CORREGIDA SEGÚN TU HARDWARE) ---
// Sensores Infrarrojos
#define SENSOR_L A8
#define SENSOR_C A9
#define SENSOR_R A10

// --- DRIVER 1 (RUEDAS DELANTERAS) ---
#define MOTOR_FL_FWD 4 // Front-Left Forward
#define MOTOR_FL_REV 5 // Front-Left Reverse
#define MOTOR_FR_FWD 9 // Front-Right Forward
#define MOTOR_FR_REV 8 // Front-Right Reverse

// --- DRIVER 2 (RUEDAS TRASERAS) ---
#define MOTOR_RL_FWD 6 // Rear-Left Forward
#define MOTOR_RL_REV 7 // Rear-Left Reverse
#define MOTOR_RR_FWD 11 // Rear-Right Forward
#define MOTOR_RR_REV 10 // Rear-Right Reverse

// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 14
#define ECHO_PIN 15

// --- CONSTANTES DE CONTROL (AJUSTADAS PARA ESTABILIDAD) ---
const float Kp = 20.0;            // Constante Proporcional (reducida para suavidad)
const float Ki = 0.0;             // Constante Integral (DESACTIVADA)
const float Kd = 25.0;            // Constante Derivativa (ajustada para el nuevo Kp)
const int baseSpeed = 150;        // Velocidad de crucero
const int obstacleThreshold = 25; // AUMENTAMOS la distancia para detectar un obstáculo (en cm)

// CONSTANTE DE COMPENSACIÓN DE MOTORES
const float LEFT_MOTOR_TRIM = 1.05;

// --- CONSTANTES PARA LOS FILTROS DE KALMAN ---
// Para el error del PID
const float KALMAN_Q_PID = 0.1;
const float KALMAN_R_PID = 4.0;
// Para el Sensor Ultrasónico
const float KALMAN_Q_US = 0.1;  // Ruido del proceso del sensor
const float KALMAN_R_US = 4.0;   // Ruido de la medición del sensor

#endif