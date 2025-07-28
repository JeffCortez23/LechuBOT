#ifndef CONFIG_H
#define CONFIG_H

// --- DEFINICIÓN DE PINES (Tu Configuración Personalizada) ---
// Sensores Infrarrojos (Conectados a pines analógicos)
#define SENSOR_L A8
#define SENSOR_C A9
#define SENSOR_R A10

// Driver 1 - Motores Izquierdos
#define MOTOR_L_F_FWD 4
#define MOTOR_L_F_REV 5
#define MOTOR_L_R_FWD 6
#define MOTOR_L_R_REV 7

// Driver 2 - Motores Derehos
#define MOTOR_R_F_FWD 9
#define MOTOR_R_F_REV 8
#define MOTOR_R_R_FWD 11
#define MOTOR_R_R_REV 10

// Sensor Ultrasónico HC-SR04
#define TRIG_PIN 14
#define ECHO_PIN 15

// --- CONSTANTES DE CONTROL (¡VALORES A AJUSTAR!) ---
const float Kp = 15.0;            // Constante Proporcional
const float Ki = 0;               // Constante Integral
const float Kd = 0;               // Constante Derivativa
const int baseSpeed = 170;        // Velocidad de crucero (0-255)
const int obstacleThreshold = 15; // Distancia para detectar un obstáculo (en cm)

// NUEVA CONSTANTE DE COMPENSACIÓN
// Si las ruedas izquierdas son lentas, aumenta este valor (ej: 1.05, 1.1).
// Si las derechas son lentas, déjalo en 1.0 y habla conmigo para invertir la lógica.
const float LEFT_MOTOR_TRIM = 1.05;

#endif