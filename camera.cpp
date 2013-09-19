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
 
  Serial1.begin(38400);
  SendResetCmd();
  delay(1000);
 /* setPicSize();
  while(Serial1.available()>0) {
    incomingbyte=Serial1.read();
  }*/
  
//  BaudRate();
//  delay(100);
//  Serial1.end();
 // Serial1.begin(57600);
  
}
void takePicture(){
   StopTakePhotoCmd();
   delay(100);
  while(Serial1.available()>0) {
    incomingbyte=Serial1.read();
  }
  SendTakePhotoCmd();
   //Serial.println("Start pic"); 
  delay(100);
  while(Serial1.available()>0) {
    incomingbyte=Serial1.read();
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
    while(Serial1.available()>0) {
      incomingbyte=Serial1.read();
      k++;
      if((k>5)&&(j<32)&&(!EndFlag)) {
        img[j]=incomingbyte;
        if((img[j-1]==0xFF)&&(img[j]==0xD9))
          EndFlag=1;                           
        j++;
        count++;
      }
    }
            
    for(j=0;j<count;j++) {   
      //if(img[j]<0x10)
        //Serial.print("0");
      //Serial.print(img[j],HEX);//HEX
      loopcnt++;
    }
    //Serial.println();
    
  //  fprintf(stderr, "Loop Count %i\n", loopcnt);
  return EndFlag;
  
}

//Send Reset command
void SendResetCmd() {
  Serial1.write((byte)0x56);
   // delay (2);
  Serial1.write((byte)0x00);
    delay (2);
  Serial1.write((byte)0x26);
   // delay (2);
  Serial1.write((byte)0x00); 
 // delay (2);  
}

//Send take picture command
void SendTakePhotoCmd() {
  Serial1.write((byte)0x56);
    delay (1);
  Serial1.write((byte)0x00);
    delay (1);
  Serial1.write((byte)0x36);
    delay (1);
  Serial1.write((byte)0x01);
    delay (1);
  Serial1.write((byte)0x00);
    delay (1);
    
  a = 0x0000; //reset so that another picture can taken
}

void FrameSize() {
  Serial1.write((byte)0x56);
   
  Serial1.write((byte)0x00);
 
  Serial1.write((byte)0x34);
  
  Serial1.write((byte)0x01);
  
  Serial1.write((byte)0x00); 
  
}
void setPicSize() {
  Serial1.write((byte)0x56);
   
  Serial1.write((byte)0x00);
 
  Serial1.write((byte)0x54);
  
  Serial1.write((byte)0x01);
  
  Serial1.write((byte)0x00); 
  
}
void BaudRate() {
 Serial1.write((byte)0x56);
   
  Serial1.write((byte)0x00);
    
 Serial1.write((byte)0x24);
  
 Serial1.write((byte)0x03);
   
  Serial1.write((byte)0x01); 
    
   Serial1.write((byte)0x1C);
   
 Serial1.write((byte)0x4C); 
    
}

//Read data
void SendReadDataCmd() {
  MH=a/0x100;
  ML=a%0x100;
      
  Serial1.write((byte)0x56);
  
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)0x32);
   
  Serial1.write((byte)0x0c);
   
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)0x0a);
    
  Serial1.write((byte)0x00);
  
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)MH);
    
  Serial1.write((byte)ML);
   
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)0x20);//0x20
   
  Serial1.write((byte)0x00);
   
  Serial1.write((byte)0x0a);
   

 a+=0x20; 
// a+=0x40;
}

void StopTakePhotoCmd() {
  Serial1.write((byte)0x56);
   
  Serial1.write((byte)0x00);
    
  Serial1.write((byte)0x36);
   
  Serial1.write((byte)0x01);
   
  Serial1.write((byte)0x03);  
   
}
