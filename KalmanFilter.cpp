#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(float q, float r) {
    _q = q;
    _r = r;
    _p = 0.0;
    _x = 0.0;
    _k = 0.0;
}

float KalmanFilter::update(float measurement) {
    _p = _p + _q;
    _k = _p / (_p + _r);
    _x = _x + _k * (measurement - _x);
    _p = (1 - _k) * _p;
    return _x;
}