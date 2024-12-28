#pragma once

#include "BasicLinearAlgebra.h"
#include <cmath>

class magCalib {
    private:
    float cenX, cenY, cenZ, rad, red;
    float valX, valY, valZ;
    float dX, dY, dZ;
    float lr = 0.01f;

    public:
    void updateCalib(float inX, float inY, float inZ){
        valX = inX;
        valY = inY;
        valZ = inZ;
        dX = valX - cenX;
        dY = valY - cenY;
        dZ = valZ - cenZ;
        red = dX * dX + dY * dY + dZ * dZ - rad * rad;
        cenX += 4.0 * lr * red * dX;
        cenY += 4.0 * lr * red * dY;
        cenZ += 4.0 * lr * red * dZ;
        rad += 4.0 * lr * red * rad;
    }

    void checkCalib(float& redIn){
        redIn = red;
    } 

};