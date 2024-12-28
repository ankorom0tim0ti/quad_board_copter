#include <Arduino.h>
#include <math.h>
#include "UDPRapper.hpp"
#include "CommunicationPacks.hpp"
#include "bmx055.hpp"
#include "SparkFun_LPS25HB_Arduino_Library.h"
#include "MadgwickAHRS.h"
#include "BasicLinearAlgebra.h"
#include "rotor.hpp"
#include "ekf.hpp"
// #include "ans.hpp"

#define SDA_PIN 5
#define SCL_PIN 6

#define DUTY_MAX 450
#define DUTY_MIN 100

#define WIFI_FLAG false

#define HISTORY_LEN 10

//initialize instance for wifi transmission
UDPRapper wifi;
PCToAvioPack recv_data;
AvioToPCPack send_data;

//initialize instance for sensor
bmx055 imu;
LPS25HB lps;
// Madgwick MadgwickFilter;

kalmanFilter ekf;
// KalmanFilter ekf;

//initialize instance for output rotor
rotor arm1;
rotor arm2;
rotor arm3;
rotor arm4;

// Flag
bool killFlag = false;
bool armFlag = false;
bool inputFlag = false;

float rc[10];
float da, de, dT, dr;
unsigned long timeCurrent;
unsigned long timeLast;
int cnt = 0;
int lost_cnt;
float accx, accy, accz;
float gyrox, gyroy, gyroz;
float magx, magy, magz;
float press;
float roll, pitch, yaw;
float throt = 60, tRoll = 0, tPitch = 0, tYaw = 0;
float duty_1, duty_2, duty_3, duty_4;
bool wifi_flag = true;

// p_gain
float rollPgain = 3;
float pitchPgain = 3;
float yawPgain = 3;

// i_gain 
float rollIgain;
float pitchIgain;
float yawIgain;

// d_gain
float rollDgain;
float pitchDgain;
float yawDgain;

float integyaw;

// difference history list
BLA::Matrix<HISTORY_LEN, 1> rollHistory;
BLA::Matrix<HISTORY_LEN, 1> pitchHistory;
BLA::Matrix<HISTORY_LEN, 1> yawHistory;

// reference list
BLA::Matrix<1, HISTORY_LEN> timeHistory;

float gyroXoffset, gyroYoffset, gyroZoffset;

int i;

void setup()
{
    rollHistory.Fill(2);
    //serial transmission start
    Serial.begin(115200);

    //I2C transmission start
    Wire1.begin(SDA_PIN, SCL_PIN);
    imu.begin(Wire1);
    lps.begin(Wire1, LPS25HB_I2C_ADDR_DEF);

    for (i = 0; i < 2000; i++) {
        delay(1);
        Serial.println("calibration is going ...");
        imu.bmx055_gyro(gyrox, gyroy, gyroz);
        gyroXoffset += gyrox;
        gyroYoffset += gyroy;
        gyroZoffset += gyroz;
    }

    gyroXoffset /= 2000.0;
    gyroYoffset /= 2000.0;
    gyroZoffset /= 2000.0;

    //Wifi transmission start
    if (WIFI_FLAG){
        wifi.setup();
    } else {
        delay(2000);
    }

    for (int i = 0; i < 10; i++)
    {
        rc[i] = 0.0;
    }

    if(WIFI_FLAG){
        while (!wifi.udp_recv())
        {
            Serial.println("waiting for wifi connection...");
        }
        wifi.recv_data(recv_data); // reading is needed to clear the buffer of WiFiudp
        Serial.println("finished setup function");
        timeCurrent = millis();
        timeLast = timeCurrent;
        lost_cnt = 0;
    }

    delay(2000);

    //initialize MadgwickFilter
    // MadgwickFilter.begin(11);

    BLA::Matrix<6, 6> Q;
    Q(0, 0) = 5.0f * 1e-1; Q(1, 1) = 5.0f * 1e-1; Q(2, 2) = 5.0f * 1e-1;
    Q(3, 3) = 5.0f * 1e-1; Q(4, 4) = 5.0f * 1e-5; Q(5, 5) = 5.0f * 1e-5;
    ekf.setrAccVal(1.0);
    ekf.setrAccDynamicVal(10000.0f);
    ekf.setrMagVal(0.10f);
    ekf.setQ(Q);
    Serial.println("Q");
    Serial.println(Q);
    ekf.setBeta(0.0033f, 0.0032f, 0.0010f);

    // Eigen::MatrixXf Q = MatrixXf::Zero(6, 6);
    // Q(0, 0) = 5.0f * 1e-1; Q(1, 1) = 5.0f * 1e-1; Q(2, 2) = 5.0f * 1e-1;
    // Q(3, 3) = 5.0f * 1e-1; Q(4, 4) = 5.0f * 1e-5; Q(5, 5) = 5.0f * 1e-5;
    // ekf.set_R_acc_val(1.0);
    // ekf.set_R_acc_val_dynamic(10000.0f);
    // ekf.set_R_mag_val(0.10f);
    // ekf.set_Q(Q);
    // ekf.set_beta(0.0033f, 0.0032f, 0.0036f);

    timeCurrent = millis();

    //setp motor
    arm1.setup(D3, 0);
    arm2.setup(D2, 2);
    arm3.setup(D10, 4);
    arm4.setup(D9, 6);


    pinMode(LED_BUILTIN, OUTPUT);//LED for checking whether wifi conection is ok or not. If you input LOW, LED will glow and vice versa.
}

void loop()
{
    timeLast = timeCurrent;
    timeCurrent = millis();

    if (WIFI_FLAG){
        if (wifi.udp_recv())
        {
            //glow lLED to show wifi connection is fine
            digitalWrite(LED_BUILTIN, LOW);

            lost_cnt = 0;
            wifi.recv_data(recv_data);
            rc[0] = recv_data.ch1; // kill switch
            rc[1] = recv_data.ch2; // arm switch
            rc[2] = recv_data.ch3; // r_stick x -1.0 ~ 1.0
            rc[3] = recv_data.ch4; // r_stick y -1.0 ~ 1.0
            rc[4] = recv_data.ch5; // r_2 stick throttle -1.0 ~ 1.0
            rc[5] = recv_data.ch6; // l_stick x -1.0 ~ 1.0
            rc[6] = recv_data.ch7; // l_stick y -1.0 ~ 1.0
            rc[7] = recv_data.ch8; // nan
            rc[8] = recv_data.ch9; // nan
            rc[9] = recv_data.ch10; // nan
            // Serial.print("ch1 = ");Serial.print(rc[0]);Serial.print(" ch2 = ");Serial.print(rc[1]);Serial.print(" ch3 = ");Serial.print(rc[2]);Serial.print(" ch4 = ");Serial.print(rc[3]);Serial.print(" ch5 = ");Serial.print(rc[4]);Serial.print(" ch6 = ");Serial.print(rc[5]);Serial.print(" ch7 = ");Serial.print(rc[6]);Serial.print(" ch8 = ");Serial.print(rc[7]);Serial.print(" ch9 = ");Serial.print(rc[8]);Serial.print(" ch10 = ");Serial.println(rc[9]);
            
        }
        else
        {
            lost_cnt += 1;

            //flick LED to show wifi conection is lost
            if (lost_cnt % 1000 <= 500){
                digitalWrite(LED_BUILTIN, LOW);
            }else{
                digitalWrite(LED_BUILTIN, HIGH);
            }
            if (lost_cnt >= 50){
                killFlag = true;
                wifi_flag = false;
                Serial.println("lost connection");
                wifi.setup();
                if (wifi.udp_recv())
                {
                    printf("reconnected\n");
                    lost_cnt = 0;
                }
            }
        }
    }

    //receive values from sensor
    imu.bmx055_acc(accx, accy, accz);
    imu.bmx055_gyro(gyrox, gyroy, gyroz);
    imu.bmx055_mag(magx, magy, magz);
    press = lps.getPressure_hPa();


    // Serial.print(" accx : "); Serial.print(accx); Serial.print(" accy : "); Serial.print(accy); Serial.print(" accz : "); Serial.print(accz); Serial.print(" gyrox : "); Serial.print(gyrox); Serial.print(" gyroy : "); Serial.print(gyroy); Serial.print(" gyroz : "); Serial.println(gyroz);

    gyrox -= gyroXoffset;
    gyroy -= gyroYoffset;
    gyroz -= gyroZoffset;

    // //calculate rotation of body
    // MadgwickFilter.updateIMU(-1.0f * gyroy, -1.0f * gyrox, -1.0 * gyroz, accy, accx, accz); // これで機体と座標軸が合う
    // roll = MadgwickFilter.getRoll();
    // pitch = MadgwickFilter.getPitch();
    // yaw  = MadgwickFilter.getYaw();

    // yaw = yaw - 180.0;

    // Serial.print("roll : ");Serial.print(roll);Serial.print(" pitch : ");Serial.print(pitch);Serial.print("yaw : ");Serial.println(yaw);

    /*
    -180 > roll > 180 , -180 > pitch > 180, -180 > yaw > 180にしてるはず
    */

    gyrox *= M_PI / 180.0f;
    gyroy *= M_PI / 180.0f;
    gyroz *= M_PI / 180.0f;

    accx *= 9.80665f;
    accy *= 9.80665f;
    accz *= 9.80665f;

    // convert
    float gyroxTemp = gyrox;
    float gyroyTemp = gyroy;
    float gyrozTemp = gyroz;
    float accxTemp = accx;
    float accyTemp = accy;
    float acczTemp = accz;
    float magxTemp = magx;
    float magyTemp = magy;
    float magzTemp = magz;
    gyrox = -1 * gyroyTemp;
    gyroy = -1 * gyroxTemp;
    gyroz = -1 * gyroyTemp;
    accx = accyTemp;
    accy = accxTemp;
    magx = magyTemp;
    magy = magxTemp;

    // debugging comment
    // Serial.print(" accx : "); Serial.print(accx); Serial.print(" accy : "); Serial.print(accy); Serial.print(" accz : "); Serial.print(accz); Serial.print(" gyrox : "); Serial.print(gyrox); Serial.print(" gyroy : "); Serial.print(gyroy); Serial.print(" gyroz : "); Serial.print(gyroz);
    

    timeLast = timeCurrent;
    timeCurrent = millis(); // get time
    float dt = float(timeCurrent - timeLast) / 1000.0f;
    Serial.print(" dT : "); Serial.println(dt);

    ekf.updateNominal(gyrox, gyroy, gyroz, dt);
    ekf.updateAcc(accx, accy, accz);
    ekf.updateMag(magx, magy, magz);

    // print estimated states
    float roll, pitch, yaw;
    ekf.computeAngles(roll, pitch, yaw);

    integyaw += gyroz;

    // ekf.update_nominal(gyrox, gyroy, gyroz, dt);
    // ekf.update_acc(accx, accy, accz);
    // ekf.update_mag(magx, magy, magz);

    // // print estimated states
    // ekf.compute_angles(roll, pitch, yaw);
    // Serial.print(" roll : ");Serial.print(roll);Serial.print(" pitch : ");Serial.print(pitch);Serial.print(" yaw : ");Serial.println(yaw);
    Serial.print(" roll : ");Serial.print(roll);Serial.print(" pitch : ");Serial.print(pitch);Serial.print(" yaw : ");Serial.println(integyaw);


    tRoll = rc[2] * 10;
    tPitch = rc[3] * 10;
    tYaw = 0;
    throt = rc[4];
    if (rc[0] == 1){
        killFlag = true;
    }
    if(rc[1] == 1){
        armFlag = true;
    }
    if(rc[7] == 1){
        inputFlag = true;
    }


    // Serial.print(" tRoll : "); Serial.print(tRoll); Serial.print(" tPitch : "); Serial.print(tPitch); Serial.print(" tYaw : "); Serial.println(tYaw);

    // error value : order ~ 100
    float inRoll = (tRoll - roll);
    float inPitch = (tPitch - pitch);
    float inYaw = (tYaw - yaw);


    // Serial.print(" inRoll : "); Serial.print(inRoll); Serial.print(" inPitch : "); Serial.print(inPitch); Serial.print(" inYaw : "); Serial.println(inYaw);



    int indexHistory = cnt % HISTORY_LEN;

    // update hitory lists
    rollHistory(indexHistory, 0) = inRoll;
    pitchHistory(indexHistory, 0) = inPitch;
    yawHistory(indexHistory, 0) = inYaw;
    timeHistory(0, indexHistory) = dt;

    // integrate value : order ~ HISTORY_LEN
    BLA::Matrix<1, 1> integRoll = timeHistory * rollHistory;
    BLA::Matrix<1, 1> integPitch = timeHistory * pitchHistory;
    BLA::Matrix<1, 1> integYaw = timeHistory * yawHistory;

   

    // variation value : order ~ 10
    float valRoll = (inRoll - rollHistory(indexHistory - 1, 0));
    float valPitch = (inPitch - pitchHistory(indexHistory - 1, 0));
    float valYaw = (inYaw - yawHistory(indexHistory - 1, 0));

    

    //calculate factor
    float rollFactor = inRoll * rollPgain + integRoll(0, 0) * rollIgain + valRoll * rollDgain;
    float pitchFactor = inPitch * pitchPgain + integPitch(0, 0) * pitchIgain + valPitch * pitchDgain;
    float yawFactor = inYaw * yawPgain + integYaw(0, 0) * yawIgain + valYaw * yawDgain;

    


    // throtlleは min 200 max 400にしたい。
    throt = throt * 80 + 200;

     //                         roll pitch yaw throt
    BLA::Matrix<4, 4> matrix = {1.0, 1.0, 1.0, 1.0,
                                -1.0, 1.0, -1.0, 1.0,
                                -1.0, -1.0, 1.0, 1.0,
                                1.0, -1.0, -1.0, 1.0};
    BLA::Matrix<4, 1> vector = {rollFactor, pitchFactor, yawFactor, throt};
    BLA::Matrix<4, 1> result = matrix * vector;

   
    // Serial.print("Matrix-vector product at "); Serial.print("inRoll : "); Serial.print(inRoll); Serial.print("inPitch : "); Serial.print(inPitch); Serial.print("inYaw : "); Serial.print(inYaw);
    // for (int i = 0; i < 4; i++) {
    //     Serial.print(" arm"); Serial.print(i + 1); Serial.print(" : "); Serial.print(result(i, 0));
    // }
    // Serial.print(" valTime : "); Serial.print(valTime);
    // Serial.print(" rollFactor : "); Serial.print(rollFactor);
    // Serial.print(" pitchFactor : "); Serial.print(pitchFactor);
    // Serial.print(" yawFactor : "); Serial.print(yawFactor);
    // Serial.print(" inRoll : "); Serial.print(inRoll);
    // Serial.print(" inPitch"); Serial.print(inPitch);
    // Serial.print(" inYaw : "); Serial.print(inYaw);
    // Serial.print(" integRoll : "); Serial.print(integRoll);
    // Serial.print(" integPitch : "); Serial.print(integPitch);
    // Serial.print(" integYaw : "); Serial.print(integYaw);
    // Serial.print(" valRoll"); Serial.print(valRoll);
    // Serial.print(" valPitch : "); Serial.print(valPitch);
    // Serial.print(" valYaw : "); Serial.print(valYaw);
    // Serial.print(" result : "); Serial.print(result);
    // Serial.println("");

    // duty_1  = result(0, 0) * 50.0 + 150;
    // duty_2  = result(1, 0) * 50.0 + 150;
    // duty_3  = result(2, 0) * 50.0 + 150;
    // duty_4  = result(3, 0) * 50.0 + 150;

    // duty_1 = inRoll * 100.0 + inPitch * -100.0 + inYaw * -100.0 + 100.0 * throt + 200.0;
    // duty_2 = inRoll * 100.0 + inPitch * 100.0 + inYaw * 100.0 + 100.0 * throt + 200.0;
    // duty_3 = inRoll * -100.0 + inPitch * 100.0 + inYaw * -100.0 + 100.0 * throt + 200.0;
    // duty_4 = inRoll * -100.0 + inPitch * -100.0 + inYaw * 100.0 + 100.0 * throt + 200.0;

    if (armFlag) {
        duty_1 = result(0, 0);
        duty_2 = result(1, 0);
        duty_3 = result(2, 0);
        duty_4 = result(3, 0);
    }

    //duty_1 = 0;
    // duty_2 = 0;
    // duty_3 = 0;
    // duty_4 = 0;

    if(duty_1 < DUTY_MIN){
        duty_1 = DUTY_MIN;
    }
    if(duty_2 < DUTY_MIN){
        duty_2 = DUTY_MIN;
    }
    if(duty_3 < DUTY_MIN){
        duty_3 = DUTY_MIN;
    }
    if(duty_4 < DUTY_MIN){
        duty_4 = DUTY_MIN;
    }
    if(duty_1 > DUTY_MAX){
        duty_1 = DUTY_MAX;
    }
    if(duty_2 > DUTY_MAX){
        duty_2 = DUTY_MAX;
    }
    if(duty_3 > DUTY_MAX){
        duty_3 = DUTY_MAX;
    }
    if(duty_4 > DUTY_MAX){
        duty_4 = DUTY_MAX;
    }

    if (killFlag){
        duty_1 = 0;
        duty_2 = 0;
        duty_3 = 0;
        duty_4 = 0;
    }



    //output to motor
    if (killFlag) {
        arm1.output(0);
        arm2.output(0);
        arm3.output(0);
        arm4.output(0);
    } else {
        if (armFlag) {
            if (inputFlag) {
                arm1.output(duty_1);
                arm2.output(duty_2);
                arm3.output(duty_3);
                arm4.output(duty_4);
            } else {
                arm1.output(DUTY_MIN);
                arm2.output(DUTY_MIN);
                arm3.output(DUTY_MIN);
                arm4.output(DUTY_MIN);
            }
            
        } else {
            arm1.output(0);
            arm2.output(0);
            arm3.output(0);
            arm4.output(0);
        }
    }
// Serial.print(" Duty1 : "); Serial.print(duty_1);Serial.print(" Duty2 : ");Serial.print(duty_2);Serial.print(" Duty3 : ");Serial.print(duty_3);Serial.print(" Duty4 : ");Serial.println(duty_4);

    

    //send data to PC
    send_data.ch1 = roll;
    send_data.ch2 = pitch;
    send_data.ch3 = yaw;
    send_data.ch4 = inRoll;
    send_data.ch5 = inPitch;
    send_data.ch6 = inYaw;
    send_data.ch7 = duty_1;
    send_data.ch8 = duty_2;
    send_data.ch9 = duty_3;
    send_data.ch10 = duty_4;
    // wifi.udp_send(send_data);
    
    
    //delay(100);

    cnt++;
}