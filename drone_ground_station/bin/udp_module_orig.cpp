//include needed library
#include "udp_module.h"
#include <iostream>
#include <pthread.h>
#include <fstream>
#include <string.h>
#include <unistd.h>
#include <sstream>
#include <vector>
#include <string>
#include "simple_udp.h"

using namespace std;

simple_udp udp;//send port
simple_udp udp1;//receive port

struct SendPack //class of send data
{
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float ch5;
    float ch6;
    float ch7;
    float ch8;
    float ch9;
    float ch10;
};

struct ReceivePack //class of receive data
{
    float ch1;
    float ch2;
    float ch3;
    float ch4;
    float ch5;
    float ch6;
    float ch7;
    float ch8;
    float ch9;
    float ch10;
};


// define functions in cython scope
extern "C" {

    // function for setup udp communicaiton
    void init_udp(const std::string& send_ip, const std::string& receive_ip, int port){
        udp.udp_init(send_ip, port);
        udp1.udp_init(receive_ip, port);
        udp1.udp_bind(true);
    }

    
    // function for send data to remote machine. this function return 0 or 1 instead of boolean value
    void send_udp(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7, float ch8, float ch9, float ch10){

        SendPack sendPack;

        sendPack.ch1 = ch1;
        sendPack.ch2 = ch2;
        sendPack.ch3 = ch3;
        sendPack.ch4 = ch4;
        sendPack.ch5 = ch5;
        sendPack.ch6 = ch6;
        sendPack.ch7 = ch7;
        sendPack.ch8 = ch8;
        sendPack.ch9 = ch9;
        sendPack.ch10 = ch10;

        udp.udp_send(sendPack);//send data to slave
    }

    int udp_flag(){
        if(udp1.udp_recv()){
            return 1;
        }else{
            return 0;
        }
    }

    // function for receive data from remote machine. this function return vector of float 
    std::vector <float> receive_udp(){
        std::vector<float> values;
        ReceivePack data;
        if (udp1.udp_recv())//check which receive data is exist or not
        {

            udp1.recv_data(data);//read data from slave
            values.push_back(data.ch1);
            values.push_back(data.ch2);
            values.push_back(data.ch3);
            values.push_back(data.ch4);
            values.push_back(data.ch5);
            values.push_back(data.ch6);
            values.push_back(data.ch7);
            values.push_back(data.ch8);
            values.push_back(data.ch9);
            values.push_back(data.ch10);

        }

        return values;
    }     
    
}
