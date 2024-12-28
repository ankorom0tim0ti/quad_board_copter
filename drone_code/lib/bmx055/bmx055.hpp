#include<Wire.h>
// BMX055 加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)

struct bmx055 {
    private:
    float xAccl;
    float yAccl;
    float zAccl;
    float xGyro;
    float yGyro;
    float zGyro;
    int   xMag;
    int   yMag;
    int   zMag;
    TwoWire *_i2cPort;

    public:
    //秋月のそのまま引っ張ってきたのを改造した。lpsのライブラリに合わせて12c通信のポートをmain関数で指定できるようにした。
    bmx055(){
    }

    void begin(TwoWire &wirePort){
        _i2cPort = &wirePort;
        _i2cPort->beginTransmission(Addr_Accl);
        _i2cPort->write(0x0F); // Select PMU_Range register
        _i2cPort->write(0x03);   // Range = +/- 2g
        _i2cPort->endTransmission();
        delay(100);

        _i2cPort->beginTransmission(Addr_Accl);
        _i2cPort->write(0x10);  // Select PMU_BW register
        _i2cPort->write(0x08);  // Bandwidth = 7.81 Hz
        _i2cPort->endTransmission();
        delay(100);

        _i2cPort->beginTransmission(Addr_Accl);
        _i2cPort->write(0x11);  // Select PMU_LPW register
        _i2cPort->write(0x00);  // Normal mode, Sleep duration = 0.5ms
        _i2cPort->endTransmission();
        delay(100);

        _i2cPort->beginTransmission(Addr_Gyro);
        _i2cPort->write(0x0F);  // Select Range register
        _i2cPort->write(0x04);  // Full scale = +/- 125 degree/s
        _i2cPort->endTransmission();
        delay(100);

        _i2cPort->beginTransmission(Addr_Gyro);
        _i2cPort->write(0x10);  // Select Bandwidth register
        _i2cPort->write(0x07);  // ODR = 100 Hz
        _i2cPort->endTransmission();
        delay(100);
        
        _i2cPort->beginTransmission(Addr_Gyro);
        _i2cPort->write(0x11);  // Select LPM1 register
        _i2cPort->write(0x00);  // Normal mode, Sleep duration = 2ms
        _i2cPort->endTransmission();
        delay(100);

        _i2cPort->beginTransmission(Addr_Mag);
        _i2cPort->write(0x4B);  // Select Mag register
        _i2cPort->write(0x83);  // Soft reset
        _i2cPort->endTransmission();
        delay(100);
        
        _i2cPort->beginTransmission(Addr_Mag);
        _i2cPort->write(0x4B);  // Select Mag register
        _i2cPort->write(0x01);  // Soft reset
        _i2cPort->endTransmission();
        delay(100);
        
        _i2cPort->beginTransmission(Addr_Mag);
        _i2cPort->write(0x4C);  // Select Mag register
        _i2cPort->write(0x00);  // Normal Mode, ODR = 10 Hz
        _i2cPort->endTransmission();

        _i2cPort->beginTransmission(Addr_Mag);
        _i2cPort->write(0x4E);  // Select Mag register
        _i2cPort->write(0x84);  // X, Y, Z-Axis enabled
        _i2cPort->endTransmission();
        
        _i2cPort->beginTransmission(Addr_Mag);
        _i2cPort->write(0x51);  // Select Mag register
        _i2cPort->write(0x04);  // No. of Repetitions for X-Y Axis = 9
        _i2cPort->endTransmission();

        _i2cPort->beginTransmission(Addr_Mag);
        _i2cPort->write(0x52);  // Select Mag register
        _i2cPort->write(16);  // No. of Repetitions for Z-Axis = 15
        _i2cPort->endTransmission();
    }

    void bmx055_acc(float& accx, float& accy, float& accz){
        unsigned int data[6];
        for (int i = 0; i < 6; i++){
            _i2cPort->beginTransmission(Addr_Accl);
            _i2cPort->write((2 + i));// Select data register
            _i2cPort->endTransmission();
            _i2cPort->requestFrom(Addr_Accl, 1);// Request 1 byte of data
            // Read 6 bytes of data
            // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
            if (_i2cPort->available() == 1)
            data[i] = _i2cPort->read();
        }
        // Convert the data to 12-bits
        xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
        if (xAccl > 2047)  xAccl -= 4096;
        yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
        if (yAccl > 2047)  yAccl -= 4096;
        zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
        if (zAccl > 2047)  zAccl -= 4096;
        accx = xAccl * 0.00098; // range = +/-2g
        accy = yAccl * 0.00098; // range = +/-2g
        accz = zAccl * 0.00098; // range = +/-2g
    }

    void bmx055_gyro(float& gyrox, float& gyroy, float& gyroz){
        unsigned int data[6];
        for (int i = 0; i < 6; i++){
            _i2cPort->beginTransmission(Addr_Gyro);
            _i2cPort->write((2 + i));    // Select data register
            _i2cPort->endTransmission();
            _i2cPort->requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
            // Read 6 bytes of data
            // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
            if (_i2cPort->available() == 1)
            data[i] = _i2cPort->read();
        }
        // Convert the data
        xGyro = (data[1] * 256) + data[0];
        if (xGyro > 32767)  xGyro -= 65536;
        yGyro = (data[3] * 256) + data[2];
        if (yGyro > 32767)  yGyro -= 65536;
        zGyro = (data[5] * 256) + data[4];
        if (zGyro > 32767)  zGyro -= 65536;

        gyrox = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
        gyroy = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
        gyroz = zGyro * 0.0038; //  Full scale = +/- 125 degree/s
        }

    void bmx055_mag(float& magx, float& magy, float& magz){
        unsigned int data[8];
        for (int i = 0; i < 8; i++){
            _i2cPort->beginTransmission(Addr_Mag);
            _i2cPort->write((0x42 + i));    // Select data register
            _i2cPort->endTransmission();
            _i2cPort->requestFrom(Addr_Mag, 1);    // Request 1 byte of data
            // Read 6 bytes of data
            // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
            if (_i2cPort->available() == 1)
            data[i] = _i2cPort->read();
        }
        // Convert the data
        xMag = ((data[1] <<5) | (data[0]>>3));
        if (xMag > 4095)  xMag -= 8192;
        yMag = ((data[3] <<5) | (data[2]>>3));
        if (yMag > 4095)  yMag -= 8192;
        zMag = ((data[5] <<7) | (data[4]>>1));
        if (zMag > 16383)  zMag -= 32768;
        magx = xMag;
        magy = yMag;
        magz = zMag;
    }
};