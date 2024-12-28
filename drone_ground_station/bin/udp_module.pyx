from libcpp.vector cimport vector
from libcpp cimport bool
from libcpp.string cimport string
cimport cython

cdef extern from "udp_module.h":
    void init_udp(string send_ip, string receive_ip, int port)
    void send_udp(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7, float ch8, float ch9, float ch10)
    int udp_flag()
    vector[float] receive_udp()

def py_init_udp(str send_ip, str receive_ip, int port):
    cdef string c_send_ip = send_ip.encode('utf-8')
    cdef string c_receive_ip = receive_ip.encode('utf-8')
    init_udp(c_send_ip, c_receive_ip, port)

def py_send_udp(float ch1, float ch2, float ch3, float ch4, float ch5, float ch6, float ch7, float ch8, float ch9, float ch10):
    send_udp(ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10)

def py_udp_flag():
    return udp_flag() != 0

def py_receive_udp():
    cdef vector[float] result = receive_udp()
    return [result[i] for i in range(result.size())]
