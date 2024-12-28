#pragma once

#include "BasicLinearAlgebra.h"
#include <cmath>

class kalmanFilter {
    private:
    BLA::Matrix<7> x;
    BLA::Matrix<3> z;
    BLA::Matrix<7, 6> B;
    BLA::Matrix<3> v;
    BLA::Matrix<6, 6> Q;
    BLA::Matrix<7, 7> P;
    BLA::Matrix<3, 3> rAcc;
    BLA::Matrix<3, 3> rAccDynamic;
    BLA::Matrix<3, 3> rMag;
    BLA::Matrix<7, 7> Fx;
    BLA::Matrix<3, 7> H;
    BLA::Matrix<7, 3> G;

    float mN, mE, mD;
    float betaX, betaY, betaZ;
    float xNorm, accNorm;

    const float GRAV = 9.80665;

    template<int i, int j>
    void setDiagonal(BLA::Matrix<i, j>& mat, float val) {

        mat.Fill(0);

        int size;

        if (i > j){size = j;} else { size = i;}
        for (int idx = 0; idx < size; idx++) {
            mat(idx, idx) = val;
        }
    }
    
    public:
    kalmanFilter();
    void updateNominal(float gyroX, float gyroY, float gyroZ, float dT);
    void updateAcc(float accX, float accY, float accZ);
    void updateMag(float magX, float magY, float magZ);
    void setQ(BLA::Matrix<6, 6> Q){this->Q = Q;};
    void setrAccVal(float val){for ( int i = 0; i < 3; i++){rAcc(i, i) = val;}};
    void setrAccDynamicVal(float val){for ( int i = 0; i < 3; i++){ rAccDynamic(i, i) = val;}};
    void setrMag(BLA::Matrix<3, 3> rMag){ this -> rMag = rMag;};
    void setrMagVal(float val){for ( int i = 0; i < 3; i++){rMag(i, i) = val;}};
    void setMagNED(float mN, float mE, float mD){ this -> mN = mN; this -> mE = mE; this -> mD = mD;};
    void setBeta(float betaX, float betaY, float betaZ){ this -> betaX = betaX; this -> betaY = betaY; this -> betaZ = betaZ;};
    void computeAngles(float& roll, float& pitch, float& yaw);
    
};