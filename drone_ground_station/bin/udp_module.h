// udp_module.h

#ifndef UDP_MODULE_H
#define UDP_MODULE_H

#include <vector>

extern "C" {
    void init_udp(const std::string& send_ip, const std::string& receive_ip, int port);
    void send_udp(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7, float ch8, float ch9, float ch10);
    int udp_flag();
    std::vector<float> receive_udp();
}

#endif // UDP_MODULE_H