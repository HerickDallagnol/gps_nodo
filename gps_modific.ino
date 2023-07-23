#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

//declara os pinos da esp
#define RXD2 16
#define TXD2 17

//define a banda do gps e cria a softwareSerial
int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RXD2, TXD2);

//inicia o nodo e um publisher com o topico gps_data
ros::NodeHandle nh;
sensor_msgs::NavSatFix gps_msg;
ros::Publisher gps_pub("gps_data", &gps_msg);

void setup() {
  Serial.begin(9600); 
  gpsSerial.begin(GPSBaud);
  nh.initNode();
  nh.advertise(gps_pub);
}

void loop() {
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("Sinal GPS não detectado");
    while (true)  
      ;
  }
}

void displayInfo() {
  if (gps.location.isValid()) {

    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    gps_msg.latitude = gps.location.lat();

    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    gps_msg.longitude = gps.location.lng();

    gps_pub.publish(&gps_msg);
    nh.spinOnce();

  } else {
    Serial.println("Sem localização");
  }
  
}