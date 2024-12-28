#pragma once

//include needed library
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

// class for udp communicaiton
class simple_udp{
    //private value
    int sock;
    struct sockaddr_in addr;
    u_long val;//for non blocking socket
    char buf[256]; // buffer

public:
    // constructor
    simple_udp(){
    }

    // function for initializing udp communicaiton
    void udp_init(std::string address, int port){
        // create socket and define address and port number
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(address.c_str());
        addr.sin_port = htons(port);
        val = 1;
    }
    
    template <typename SendType>
    void udp_send(const SendType& send_data){
        union {
            SendType data;
            char c[256];
        } sendUnion;
        sendUnion.data = send_data;
        sendto(sock, sendUnion.c, sizeof(sendUnion.c), 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    // function for bind address of remote machine
    void udp_bind(bool nonblock){
        bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
        if (nonblock){
            ioctl(sock, FIONBIO, &val);
        }
    }

    // function for judging data is comming from remote machine
    bool udp_recv(){
        memset(buf, 0, sizeof(buf));
        if (recv(sock, buf, sizeof(buf), 0) >= 1){
            return true;
        }else{
            return false;
        }
    }

    // fucntion for store receivede data
    template <typename ReceiveType>
    void recv_data(ReceiveType& receive_data){
        union{
            ReceiveType data;
            char c[256];
        } readUnion;
        memcpy(readUnion.c, buf, sizeof(buf));
        receive_data = readUnion.data;
    }

    //function for receive data
    void udp_recv(char *buf, int size){
        memset(buf, 0, size);
        recv(sock, buf, size, 0);
    }

    // deconstructor
    ~simple_udp(){
        close(sock);
    }
};
