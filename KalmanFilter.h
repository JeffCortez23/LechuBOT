#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h>

class KalmanFilter {
public:
    KalmanFilter(float q, float r);
    float update(float measurement);

private:
    float _q;
    float _r;
    float _p;
    float _x;
    float _k;
};

#endif