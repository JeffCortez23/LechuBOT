#include "KalmanFilter.h"
#include <Arduino.h>

// Constructor
KalmanFilter::KalmanFilter(float q, float r) {
    _q = q; // Ruido del proceso: Cuánto esperamos que el estado cambie por sí solo.
    _r = r; // Ruido de la medición: Cuánto confiamos en la lectura del sensor.
    _p = 0.0; // Error de estimación inicial
    _x = 0.0; // Estado inicial
    _k = 0.0; // Ganancia de Kalman inicial
}

// Función de actualización
float KalmanFilter::update(float measurement) {
    // Predicción
    _p = _p + _q;

    // Medición (fase de actualización)
    _k = _p / (_p + _r);
    _x = _x + _k * (measurement - _x);
    _p = (1 - _k) * _p;

    return _x;
}