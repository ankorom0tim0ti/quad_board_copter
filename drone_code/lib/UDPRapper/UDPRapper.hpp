#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>


class UDPRapper{
    private:
        const char* ssid;
        const char* password;
        const int localPort;
        const IPAddress ip;
        const IPAddress subnet;
        const IPAddress remote_ip;
        WiFiUDP udp_;
        char buf[256];

    public:
        UDPRapper()
        : ssid("esp32_wifi"), password("esp32-wifi"), localPort(5000), ip(192, 168, 20, 2), subnet(255, 255, 255, 0), remote_ip(192, 168, 20, 3)
        {
        }

        void setup(){
            Serial.println("starting wifi setup");
            WiFi.softAP(ssid, password);
            //while( WiFi.status() != WL_CONNECTED) {
            //}
            delay(100);
            WiFi.softAPConfig(ip, ip, subnet);
            IPAddress myIP = WiFi.softAPIP();  // WiFi.softAPIP()でWiFi起動
            udp_.begin(localPort);
            Serial.println("finished wifi setup");
        }

        bool udp_recv(){
            if (udp_.parsePacket() > 1){
                return true;
            }else{
                return false;
            }
        }

        template <typename ReceiveType>
        void recv_data(ReceiveType& receive_data){
            union{
                ReceiveType data;
                u_int8_t c[256];
            } readUnion;
            udp_.read(buf, 256);
            memcpy(readUnion.c, buf, sizeof(buf));
            receive_data = readUnion.data;
        }

        template <typename SendType>
        void udp_send(const SendType& send_data){
            udp_.beginPacket(remote_ip, localPort);//remote_port = localPort
            union {
                SendType data;
                u_int8_t c[256];
            } sendUnion;
            sendUnion.data = send_data;
            udp_.write(sendUnion.c, sizeof(send_data));
            udp_.endPacket();
        }

};