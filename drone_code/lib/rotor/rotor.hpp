#include <Arduino.h>

struct rotor {
private:
    int pin ;
    int channel ;
    int freq  = 39000;
    int bit_num  = 10;
    int duty;
    int duty_max = 600;
    int duty_min = 0;

public:
    rotor(){

    }
    void setup(int pin_in, int channel_in){
        pin = pin_in;
        channel = channel_in;
        pinMode( pin, OUTPUT);
        ledcSetup(channel, freq, bit_num);
        ledcAttachPin(pin, channel);
    }
    
    void output(float duty_bit){
        if (duty_bit > duty_max){
            duty_bit = duty_max;
        }
        if (duty_bit < duty_min){
            duty_bit = duty_min;
        }
        ledcWrite(channel, int(duty_bit));
    }
};