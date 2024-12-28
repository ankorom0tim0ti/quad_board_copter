#include "ekf.hpp"

kalmanFilter::kalmanFilter(){
    setDiagonal(B, 1.0);
    x.Fill(0);
    x(0) = 1.0;
    v.Fill(0);
    setDiagonal(Q, 1.0);
    setDiagonal(P, 1.0);
    setDiagonal(rAcc, 1.0);
    setDiagonal(rAccDynamic, 1.0);
    setDiagonal(rMag, 1.0);
}

void kalmanFilter::updateNominal(float gyroX, float gyroY, float gyroZ, float dT){
    x(0) += 0.50 * (-(gyroX - x(4)) * x(1) - (gyroY - x(5)) * x(2) - (gyroZ - x(6)) * x(3)) * dT;
    x(1) += 0.50 * ( (gyroX - x(4)) * x(0) + (gyroZ - x(6)) * x(2) - (gyroY - x(5)) * x(3)) * dT;
    x(2) += 0.50 * ( (gyroY - x(5)) * x(0) - (gyroZ - x(6)) * x(1) + (gyroX - x(4)) * x(3)) * dT;
    x(3) += 0.50 * ( (gyroZ - x(6)) * x(0) + (gyroY - x(5)) * x(1) - (gyroX - x(4)) * x(2)) * dT;
    x(4) += -betaX * x(4) * dT;
    x(5) += -betaY * x(5) * dT;
    x(6) += -betaZ * x(6) * dT;

    //test
    // Serial.println("test_a");
    // Serial.println("B");
    // Serial.print(B);
    // Serial.println("Q");
    // Serial.print(Q);
    // Serial.println("P");
    // Serial.print(P);
    // Serial.println("rAcc");
    // Serial.print(rAcc);
    // Serial.println("rAccDydynamic");
    // Serial.print(rAccDynamic);
    // Serial.println("");


    xNorm = sqrtf(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    // Serial.print("quaternion updateNominal_a : "); Serial.println(x);
    // Serial.print(" xNorm updateNominal_a : ");Serial.println(xNorm);

    x(0) /= xNorm;
    x(1) /= xNorm;
    x(2) /= xNorm;
    x(3) /= xNorm;
    
    setDiagonal(Fx, 1.0);

    Fx(0, 1) += -0.5 * (gyroX - x(4)) * dT;
    Fx(0, 2) += -0.5 * (gyroY - x(5)) * dT;
    Fx(0, 3) += -0.5 * (gyroZ - x(6)) * dT;
    Fx(0, 4) +=  0.5 * x(1) * dT;
    Fx(0, 5) +=  0.5 * x(2) * dT;
    Fx(0, 6) +=  0.5 * x(3) * dT;

    Fx(1, 0) +=  0.5 * (gyroX - x(4)) * dT;
    Fx(1, 2) +=  0.5 * (gyroZ - x(6)) * dT;
    Fx(1, 3) += -0.5 * (gyroY - x(5)) * dT;
    Fx(1, 4) += -0.5 * x(0) * dT;
    Fx(1, 5) +=  0.5 * x(3) * dT;
    Fx(1, 6) += -0.5 * x(2) * dT;

    //x(2, 0) = x(2) + 0.5*( (gyro_y-x(5))*x(0) -(gyro_z-x(6))*x(1) +(gyro_x-x(4))*x(3))*dt; <-miss!
    Fx(2, 0) +=  0.5 * (gyroY - x(5)) * dT;
    Fx(2, 1) += -0.5 * (gyroZ - x(6)) * dT;
    Fx(2, 3) +=  0.5 * (gyroX - x(4)) * dT;
    Fx(2, 4) += -0.5 * x(3) * dT;
    Fx(2, 5) += -0.5 * x(0) * dT;
    Fx(2, 6) +=  0.5 * x(1) * dT;
  
    //x(3, 0) = x(3) + 0.5*( (gyro_z-x(6))*x(0) +(gyro_y-x(5))*x(1) -(gyro_x-x(4))*x(2))*dt;
    Fx(3, 0) +=  0.5 * (gyroZ - x(6)) * dT;
    Fx(3, 1) +=  0.5 * (gyroY - x(5)) * dT;
    Fx(3, 2) += -0.5 * (gyroX - x(4)) * dT;
    Fx(3, 4) +=  0.5 * x(2) * dT;
    Fx(3, 5) += -0.5 * x(1) * dT;
    Fx(3, 6) += -0.5 * x(0) * dT;

    Fx(4, 4) += -betaX * dT;
    Fx(5, 5) += -betaY * dT;
    Fx(6, 6) += -betaZ * dT;

    B(0, 0) = -0.5 * x(1);
    B(0, 1) = -0.5 * x(2);
    B(0, 2) = -0.5 * x(3);

    B(1, 0) =  0.5 * x(0);
    B(1, 1) = -0.5 * x(3);
    B(1, 2) =  0.5 * x(2);

    B(2, 0) =  0.5 * x(3);
    B(2, 1) =  0.5 * x(0);
    B(2, 2) = -0.5 * x(1);

    B(3, 0) = -0.5 * x(0);
    B(3, 1) =  0.5 * x(1);
    B(3, 2) =  0.5 * x(2);

    this -> P = Fx * P * ~Fx + B * Q * ~B;
}

void kalmanFilter::updateAcc(float accX, float accY, float accZ){
    accNorm = accX * accX + accY * accY + accZ * accZ;

    H.Fill(0);
    H(0, 0) = -2.0 * x(2) * GRAV;
    H(0, 1) =  2.0 * x(3) * GRAV;
    H(0, 2) = -2.0 * x(0) * GRAV;
    H(0, 3) =  2.0 * x(1) * GRAV;

    H(1, 0) = 2.0 * x(1) * GRAV;
    H(1, 1) = 2.0 * x(0) * GRAV;
    H(1, 2) = 2.0 * x(3) * GRAV;
    H(1, 3) = 2.0 * x(2) * GRAV;

    H(2, 0) =  2.0 * x(0) * GRAV;
    H(2, 1) = -2.0 * x(1) * GRAV;
    H(2, 2) = -2.0 * x(2) * GRAV;
    H(2, 3) =  2.0 * x(3) * GRAV;

    if(abs(accNorm / (GRAV * GRAV) - 1) < 0.20) {
        G = P * ~H * Inverse(H * P * ~H + rAcc);
    } else {
        G = P * ~H * Inverse(H * P * ~H + rAccDynamic);
    }

    P = P - G * H * P;

    z(0) = accX - v(0) - 2.0f * (x(1)*x(3) - x(0)*x(2)) * GRAV;
    z(1) = accY - v(1) - 2.0f * (x(2)*x(3) + x(0)*x(1)) * GRAV;
    z(2) = accZ - v(2) - (x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) * GRAV;

    x += G * z;
    xNorm = sqrtf(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    x(0) /= xNorm;
    x(1) /= xNorm;
    x(2) /= xNorm;
    x(3) /= xNorm;
}

void kalmanFilter::updateMag(float magX, float magY, float magZ){
    H.Fill(0);
    H(0, 0) = 2.0 * ( x(0) * mN + x(3) * mE - x(2) * mD);
    H(0, 1) = 2.0 * ( x(1) * mN + x(2) * mE + x(3) * mD);
    H(0, 2) = 2.0 * (-x(2) * mN + x(1) * mE - x(0) * mD);
    H(0, 3) = 2.0 * (-x(3) * mN + x(0) * mE + x(1) * mD);

    H(1, 0) = 2.0 * (-x(3) * mN + x(0) * mE + x(1) * mD);
    H(1, 1) = 2.0 * ( x(2) * mN - x(1) * mE + x(0) * mD);
    H(1, 2) = 2.0 * ( x(1) * mN + x(2) * mE + x(3) * mD);
    H(1, 3) = 2.0 * (-x(0) * mN - x(3) * mE + x(2) * mD);

    H(2, 0) = 2.0 * ( x(2) * mN - x(1) * mE + x(0) * mD);
    H(2, 1) = 2.0 * ( x(3) * mN - x(0) * mE - x(1) * mD);
    H(2, 2) = 2.0 * ( x(0) * mN + x(3) * mE - x(2) * mD);
    H(2, 3) = 2.0 * ( x(1) * mN + x(2) * mE + x(3) * mD);

    G = P * ~H * Inverse(H * P * ~H + rAcc);

    P = P - G * H * P;

    z(0) = magX - v(0) - (x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3)) * mN - 2.0f * (x(1)*x(2) + x(0)*x(3)) * mE - 2.0f * (x(1)*x(3) - x(0)*x(2)) * mD;
    z(1) = magY - v(1) - 2.0f * (x(1)*x(2) - x(0)*x(3)) * mN - (x(0)*x(0) - x(1)*x(1) + x(2)*x(2) - x(3)*x(3)) * mE - 2.0f * (x(2)*x(3) + x(0)*x(1)) * mD;
    z(2) = magZ - v(2) - 2.0f * (x(1)*x(3) + x(0)*x(2)) * mN + 2.0f * (x(2)*x(3) - x(0)*x(1)) * mE + (x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3)) * mD;
    Serial.print(H);
    x += G * z;

    xNorm = sqrtf(x(0)*x(0) + x(1)*x(1) + x(2)*x(2) + x(3)*x(3));
    x(0) /= xNorm;
    x(1) /= xNorm;
    x(2) /= xNorm;
    x(3) /= xNorm;
}

void kalmanFilter::computeAngles(float& roll, float& pitch, float& yaw){
    roll  = atan2f(2.0 * (x(2)*x(3) + x(0)*x(1)), x(0)*x(0) - x(1)*x(1) - x(2)*x(2) + x(3)*x(3));
    pitch =  asinf(2.0 * (x(0)*x(2) - x(1)*x(3)));
    yaw   = atan2f(2.0 * (x(1)*x(2) + x(0)*x(3)), x(0)*x(0) + x(1)*x(1) - x(2)*x(2) - x(3)*x(3));
}