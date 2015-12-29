/*
    set Ethernet Ip = 10.42.0.21
                 gateway(10.42.0.1);
                 
                 
    * Ethernet shield attached to pins 10, 11, 12, 13
*/

#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = { //98:4F:EE:00:2E:98
  0x98, 0x4F, 0xEE, 0x00, 0x2E, 0x98 };
IPAddress ip(10,42,0, 21);
IPAddress gateway(10,42,0, 1);
IPAddress subnet(255, 255, 255, 00);

//ssh defaults to port 22
EthernetServer server(22);
void setup() {
  // put your setup code here, to run once:
  Ethernet.begin(mac, ip, gateway, subnet);
}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
