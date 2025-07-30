#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter {
public:
    // Constructor: inicializa el filtro con los parámetros de ruido
    KalmanFilter(float q, float r);

    // Actualiza el filtro con una nueva medición y devuelve el valor filtrado
    float update(float measurement);

private:
    float _q; // Ruido del proceso
    float _r; // Ruido de la medición
    float _p; // Estimación del error
    float _x; // Valor estimado
    float _k; // Ganancia de Kalman
};

#endif