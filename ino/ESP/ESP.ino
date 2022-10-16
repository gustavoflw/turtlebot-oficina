#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <std_msgs/Bool.h>                // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Bool.html
#include <std_msgs/Int32.h>               // http://docs.ros.org/en/lunar/api/std_msgs/html/msg/Int32.html
#include <std_msgs/Float64.h>             // http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float64.html
#include <sensor_msgs/Temperature.h>      // https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html
#include <sensor_msgs/RelativeHumidity.h> // http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/RelativeHumidity.html
#include <geometry_msgs/Twist.h>          // http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html
#include <geometry_msgs/Vector3.h>        // http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html

#include "SerialCom.hpp"
#include "MyWiFi.hpp"

/* WIFI */
const char *wifi_ssid      = "elonmusk";
const char *wifi_password  = "elonmusk";
int        rosmaster_port  = 11411;
uint8_t    newMACAddress[] = {0x5C, 0xE8, 0x83, 0x36, 0x9A, 0xC3};
IPAddress  gateway(192, 168, 4, 1);
IPAddress  rosmaster(gateway[0], gateway[1], gateway[2], 31        );
IPAddress  localIP  (gateway[0], gateway[1], gateway[2], 32        );
IPAddress  dns      (gateway[0], gateway[1], gateway[2], gateway[3]);
IPAddress  subnet   (       255,        255,        255,        0  );

/* SERIAL */
// const byte numChars = 64;
// char       receivedChars [numChars];
// char       tempChars     [numChars];
// char       messageFromUno[numChars] = {0};
// boolean    newData = false;

void setup() 
{
  // Wifi
  MyWiFi::SetupClient(wifi_ssid, wifi_password, localIP, gateway, subnet, dns);
  pinMode           (LED_BUILTIN, OUTPUT);
  digitalWrite      (LED_BUILTIN, LOW   ); // LOW acende o led (??????)
}

void loop() 
{

}
