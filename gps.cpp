#include <stdio.h>
#include <arduino.h>
#include "gps.h"
//
double latitude, longitude,alt,speedkmh;
uint8_t time_sec,time_hour,time_minute;
uint32_t num_satellites;


unsigned long chars;
unsigned short sentences, failed_checksum;

char buf[32]; 
void initGPS() {

  Serial1.begin(9600);
  // put your setup code here, to run once:

}
String getlong(){
  sprintf(buf,"%+09.4f",longitude);
  return buf;
}
String getlat(){
  sprintf(buf,"%+08.4f",latitude);
  return buf;
}
String getalt(){
  sprintf(buf,"%05.0f",alt);
   return buf;
}
String getgpsTime(){
   String gpstime;
   
   sprintf(buf,"%02d:",time_hour);
   gpstime=buf;
   sprintf(buf,"%02d:",time_minute);
   gpstime=gpstime+buf;
   sprintf(buf,"%02d",time_sec);
   gpstime=gpstime+buf;
   return gpstime;
}
String getnumsatellites(){
   sprintf(buf,"%02d",num_satellites);
   return buf;
}
String getspeed(){
  sprintf(buf,"%03.0f",speedkmh);
  return buf;
}
String getage(){
  if(gps.location.age() > 9000)
    return "0";else return "1";
}

boolean getGPSdata() {
  // put your main code here, to run repeatedly: 
  while (Serial1.available())
  {
    int c = Serial1.read();
    //   Serial.write(c);
    if (gps.encode(c))
    {
      if (gps.location.isValid())
      {
        // retrieves +/- lat/long in 100000ths of a degree
        latitude= gps.location.lat();
        longitude = gps.location.lng();
        alt =   gps.altitude.meters();
        time_sec = gps.time.second();
        time_minute = gps.time.minute();
        time_hour = gps.time.hour();
        num_satellites= gps.satellites.value();
        speedkmh = gps.speed.kmph();



       // returns speed in 100ths of a knot
      // speed = gps.speed();

        // course in 100ths of a degree
       // course = gps.course();
      return true;
      }
      else
      {
        Serial.print(("INVALID"));
        return false;
      }
    }
    return false;
  }
}


