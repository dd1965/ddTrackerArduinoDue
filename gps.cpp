#include <stdio.h>
#include <arduino.h>
#include "gps.h"
//
double latitude, longitude,alt,speedkmh;
uint8_t time_sec,time_hour,time_minute;
uint32_t num_satellites;
char  locator[9];

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
    return "0";
  else return "1";
}
String getGrid(){  
  return locator;
}
void tbc_calculate_grid_square(double latin, double lonin)
{
  double latremainder;
  double lonremainder;
  // uint32_t lat;
  // uint32_t long;
  latin += 90;
  lonin +=180;
 

  //Calculate lon part of grid square
  locator[0] = (char)('A' + ((char)((lonin/ 20))));
  double integerpart;
  lonremainder = (modf(lonin/20,&integerpart))* 20;
  locator[2] = (char)((((lonremainder/ 2))+'0'));

  lonremainder = (modf(lonremainder/2,&integerpart))* 2;
  locator[4]= (char)('A' + ((char)((lonremainder/ 0.083333))));
  lonremainder = (modf(lonremainder/0.083333,&integerpart))* 0.083333;
  // lonremainder= lonremainder%0.083333;
  locator[6] = (char)((((lonremainder / 0.008333)) + '0'));

  //Calculate lat part of grid square
  locator[1] = (char)('A' + ((char)(((latin) / 10))));
  // latremainder = latin % 10;
  latremainder = (modf(latin/10,&integerpart))* 10;
  locator[3] = (char)((((latremainder / 1)) + '0'));
  //  latremainder = latremainder % 1;
  latremainder = (modf(latremainder/1,&integerpart))* 1;
  locator[5] = (char)('A' + ((char)(((latremainder) / 0.0416665))));
  //   latremainder = latremainder % 0.0416665;
  latremainder = (modf(latremainder/0.0416665,&integerpart))*0.0416665;
  locator[7] = (char)((((latremainder / 0.004166)) + '0'));
   locator[8] = '\0';
 /* string s = new string(locator);
  Console.WriteLine(s);
  */
  Serial.print("Locator ");
  Serial.println(locator);
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
        tbc_calculate_grid_square( latitude, longitude);


        // returns speed in 100ths of a knot
        // speed = gps.speed();

        // course in 100ths of a degree
        // course = gps.course();
        return true;
      }
      else
      {
        //Serial.print(("INVALID"));
        return false;
      }
    }
    return false;
  }
}



