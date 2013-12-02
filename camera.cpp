#include <arduino.h>
#include "camera.h"
//#include <SoftwareSerial.h> 

byte incomingbyte;




int a=0x0000,  //Read Starting address     
    j=0,
    k=0,
    count=0;
uint8_t MH,ML;
boolean EndFlag=0;
long loopcnt=0;

void initCamera() { 
 
  Serial2.begin(38400);
  SendResetCmd();
  delay(1000);
 // setPicSize();
  while(Serial2.available()>0) {
    incomingbyte=Serial2.read();
   // Serial.print(incomingbyte,HEX);
  //  Serial.print(" ");
  }
  
//  BaudRate();
//  delay(100);
//  Serial2.end();
 // Serial2.begin(57600);
  
}
void takePicture(){
   StopTakePhotoCmd();
   delay(100);
  while(Serial2.available()>0) {
    incomingbyte=Serial2.read();
 //   Serial.print(incomingbyte,HEX);
  //  Serial.print(" ");
  }
  SendTakePhotoCmd();
//  Serial.println("Start pic"); 
  delay(100);
  while(Serial2.available()>0) {
    incomingbyte=Serial2.read();
 //   Serial.print(incomingbyte,HEX);
 //   Serial.print(" ");
  }
  loopcnt=0;
  EndFlag=0;
}

int getPicture(uint8_t *img) {
 
  // SendResetCmd();
   // SendResetCmd();
    //BaudRate();  
  //byte b[32];
      
    j=0;
    k=0;
    count=0;
    SendReadDataCmd();
           
    delay(150); //try going up
    while(Serial2.available()>0) {
      incomingbyte=Serial2.read();
     /* Serial.print(incomingbyte,HEX);
      Serial.print(" ");*/
      k++;
      if((k>5)&&(j<32)&&(!EndFlag)) {
        img[j]=incomingbyte;
        if((img[j-1]==0xFF)&&(img[j]==0xD9))
          EndFlag=1;                           
        j++;
        count++;
      }
    }
            
   /* for(j=0;j<count;j++) {   
    if(img[j]<0x10)
      Serial.print("0");
      Serial.print(img[j],HEX);//HEX
      loopcnt++;
    }
    Serial.println("Exiting getPicture");*/
    
  //  fprintf(stderr, "Loop Count %i\n", loopcnt);
  return EndFlag;
  
}

//Send Reset command
void SendResetCmd() {
  Serial2.write((byte)0x56);
   // delay (2);
  Serial2.write((byte)0x00);
    delay (2);
  Serial2.write((byte)0x26);
   // delay (2);
  Serial2.write((byte)0x00); 
 // delay (2);  
}

//Send take picture command
void SendTakePhotoCmd() {
  Serial2.write((byte)0x56);
    delay (1);
  Serial2.write((byte)0x00);
    delay (1);
  Serial2.write((byte)0x36);
    delay (1);
  Serial2.write((byte)0x01);
    delay (1);
  Serial2.write((byte)0x00);
    delay (1);
    
  a = 0x0000; //reset so that another picture can taken
}

void FrameSize() {
  Serial2.write((byte)0x56);
   
  Serial2.write((byte)0x00);
 
  Serial2.write((byte)0x34);
  
  Serial2.write((byte)0x01);
  
  Serial2.write((byte)0x00); 
  
}
void setPicSize() {
  Serial2.write((byte)0x56);
   
  Serial2.write((byte)0x00);
 
  Serial2.write((byte)0x54);
  
  Serial2.write((byte)0x01);
  
  Serial2.write((byte)0x00); 
  
}
void BaudRate() {
 Serial2.write((byte)0x56);
   
  Serial2.write((byte)0x00);
    
 Serial2.write((byte)0x24);
  
 Serial2.write((byte)0x03);
   
  Serial2.write((byte)0x01); 
    
   Serial2.write((byte)0x1C);
   
 Serial2.write((byte)0x4C); 
    
}

//Read data
void SendReadDataCmd() {
  MH=a/0x100;
  ML=a%0x100;
      
  Serial2.write((byte)0x56);
  
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)0x32);
   
  Serial2.write((byte)0x0c);
   
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)0x0a);
    
  Serial2.write((byte)0x00);
  
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)MH);
    
  Serial2.write((byte)ML);
   
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)0x20);//0x20
   
  Serial2.write((byte)0x00);
   
  Serial2.write((byte)0x0a);
   

 a+=0x20; 
// a+=0x40;
}

void StopTakePhotoCmd() {
  Serial2.write((byte)0x56);
   
  Serial2.write((byte)0x00);
    
  Serial2.write((byte)0x36);
   
  Serial2.write((byte)0x01);
   
  Serial2.write((byte)0x03);  
   
}
