//This file will just contain functions to turn xbox controller joystick values into a 360 degree direction value 0-359 or get the magnitude
#pragma once

#include <math.h>

float angleCalc(float x, float y) {
    float angle;
    if (x == 0 && y == 0)
        return -1;
    if (y == 0)
        angle = 90.0;
    else
        angle = atanf(abs(x)/abs(y)); //may be asinf, not quite sure yet
    if (x < 0 && y < 0) {
        angle += 180;
    }
    else if (x < 0) {
        angle += 180 + (2 * (90 - angle));
        if (y < 0)
            angle -= 2 * (90 - angle);
    }
    else if (y < 0)
        angle += 2 * (90 - angle);
    return angle;
}

float distCalc(float x, float y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}