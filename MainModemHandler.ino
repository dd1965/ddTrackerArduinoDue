
#include "config.h"
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>
extern int sendMorse(char *msg,int Length);
//#include "ssdv.h"
#include "configGPS.h"
#include "rs8.h"
#include "camera.h"

#include <stdio.h>
#include <ByteBuffer.h>
#include <SPI.h>
//TinyGPSPlus gps;
// Reuses timer setup and sinewave creation based on a work at rcarduino.blogspot.com & RCArduino DDS Sinewave for Arduino Due.

// This is a telemetry sender for 1200 baud created by VK3TBC.
// This is a test harness only and not complete. No GPS interface yet. Can send pictures at 1200 baud with dummy telemetry packet.
// Features 1200 baud digital modulator & experimental 4800 baud modulator
// Sample rate is 84Khz to the audio DAC. 1023 point sine wave (10bit)
// Constructs a simple frame for Telemetry and sends it.
// Added a buffer for audio samples that need to be transmitted.
// Original licence condition kept.

// RCArduino DDS Sinewave by RCArduino is licensed under a Creative Commons Attribution 3.0 Unported License.
// For helpful background information on Arduino Due Timer Configuration, refer to the following link
// thanks to Sebastian Vik
// http://arduino.cc/forum/index.php?action=post;topic=130423.15;num_replies=20

// For background information on the DDS Technique see
// http://interface.khm.de/index.php/lab/experiments/arduino-dds-sinewave-generator/

// For audio sketches making extensive use of DDS Techniques, search the RCArduino Blog
// for the tags Audio or synth

// These are the clock frequencies available to the timers /2,/8,/32,/128
// 84Mhz/2 = 42.000 MHz
// 84Mhz/8 = 10.500 MHz
// 84Mhz/32 = 2.625 MHz
// 84Mhz/128 = 656.250 KHz
// for 88khz sample 42Mhz/88Khz = 500 fits evenly.
// use a simple filter on the DAC.

// Experimental 4800 baud raised cosine filter constants
uint16_t baud_4800_table[] ={
  119,348,624,937,1274,1623,1969,2296,2591,
  2845,3086,3286,3443,3552,3614,3626,3589,3504,3370,3191,2970,2724,2448,2135,1798,1449,1103,776,481,227,23,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3088,3072,3072,3072,3072,3072,3072,3072,3072,3072,
  3108,3157,3214,3272,3322,3356,3363,3333,3257,3132};
// 4800 Baud interrupt handler functions
volatile byte previoussentbit = 0;
volatile int waveIndex=0;
volatile  uint32_t txshreg=0;
volatile byte transmiton=1;
volatile boolean timerstarted=false;
volatile uint32_t timer=0;
volatile uint16_t timer10ms=0;
volatile boolean send_morse=false;
ByteBuffer send_buffer;
//End 4800 baud

// the phase accumulator points to the current sample in our wavetable
uint32_t ulPhaseAccumulator = 0;
// the phase increment controls the rate at which we move through the wave table
// higher values = higher frequencies
volatile uint32_t ulPhaseIncrement = 0;   // 32 bit phase increment, see below
volatile int idle=0;
volatile int bufcnt=0;
volatile int bti = 0;
volatile int K=8;
volatile  byte inbytei;
int sendtone = 0;
String startHdr="$CQ DE ";
byte hdrbyte[4];
String test="VK3TBC-1,0000,00:00:00,0.0,0.0,0,0,0,0,0,0,0,Test";
String part2="VK3TBC-1,0000,";
String part3=",000,000,000,Test*";
String tempin ="000";
String tempout ="000";
String voltage = "000";
//String speedveld = "0,0,";
String separator = ",";
char *callsign ="VK3TBC";
String seqnostr ="0000";
byte testStr[256];
byte eol[2];

short seq=0;
char seqimg=0;
char bufseq[32]; 
byte flags[80];
byte encodeArray[12];
byte crcbyte[4];
ushort crc;
String crchex;
String seqNumStr;
int led = 13;
//Interleaver
#define numbits 2048
#define numbytes = 256
byte symbols_interleaved[numbits];
byte symbols[numbits];
byte bitin;
byte cbyte;
// full waveform = 0 to SAMPLES_PER_CYCLE
// Phase Increment for 1 Hz =(SAMPLES_PER_CYCLE_FIXEDPOINT/SAMPLE_RATE) = 1Hz
// Phase Increment for frequency F = (SAMPLES_PER_CYCLE/SAMPLE_RATE)*F
#define SAMPLE_RATE 84000.0//84000
#define SAMPLES_PER_CYCLE 1020
#define SAMPLES_PER_CYCLE_FIXEDPOINT (SAMPLES_PER_CYCLE<<20)
#define TICKS_PER_CYCLE (float)((float)SAMPLES_PER_CYCLE_FIXEDPOINT/(float)SAMPLE_RATE)
#define FIVE_SEC_TIMER 10*SAMPLE_RATE;
uint32_t timer5 = FIVE_SEC_TIMER;
// to represent 1020 we need 10 bits
// Our fixed point format will be 10P22 = 32 bits

// Create a table to hold the phase increments we need to generate tone frequencies at our 84Khz sample rate
#define No_of_tones 2
uint32_t toneRegister[No_of_tones];


//Define One Wire routines
#define ONE_WIRE_BUS 2
#define ONE_WIRE_BUS2 3
OneWire oneWire(ONE_WIRE_BUS);
OneWire oneWireo(ONE_WIRE_BUS2);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DallasTemperature sensorso(&oneWireo);
// arrays to hold device address
DeviceAddress insideThermometer;
DeviceAddress outsideThermometer;
char tbuf[32];
char cwstring[56];

// fill the table with the phase increment values we require to generate the tone 1200Hz and 2200Hz
void createToneIndex()
{
    toneRegister[0] = 1200*TICKS_PER_CYCLE;
    toneRegister[1] = 2200*TICKS_PER_CYCLE;
}

// Create a table to hold pre computed sinewave, the table has a resolution of 600 samples
#define WAVE_SAMPLES 1020
// default int is 32 bit, in most cases its best to use uint32_t but for large arrays its better to use smaller
// data types if possible, here we are storing 12 bit samples in 16 bit ints
uint16_t nSineTable[WAVE_SAMPLES];

// create the individual samples for our sinewave table
void createSineTable()
{
  for(uint32_t nIndex = 0;nIndex < WAVE_SAMPLES;nIndex++)
  {
    // normalised to 12 bit range 0-4095
    nSineTable[nIndex] = (uint16_t)  (((1+sin(((2.0*PI)/WAVE_SAMPLES)*nIndex))*4095.0)/2);
    // Serial.println(nSineTable[nIndex]);
  }
}

void setup()
{
  pinMode(led, OUTPUT);  
  digitalWrite(led, HIGH);
  delay(5000);
  digitalWrite(led, LOW);
  Serial.begin(9600);
  send_buffer.init(10240);
  encodeGold(0);
  for(int f=0;f<2;f++) eol[f]=0x0A;
  for(int f=0;f<4;f++) hdrbyte[f]=0x24;
  createToneIndex();
  createSineTable();
  for(int f=0;f<20;f++) flags[f]=0x7E;
  initCamera();
  /* turn on the timer clock in the power management controller */
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC4);

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */TC1,/* channel */1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_SetRC(TC1, 1, 500); // sets <> 44.1 Khz interrupt rate // 500 for 84Khz //438 for 4800 baud ie 42Mhz/96Khz
  TC_Start(TC1, 1);

  // enable timer interrupts on the timer
  TC1->TC_CHANNEL[1].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[1].TC_IDR=~TC_IER_CPCS;

  /* Enable the interrupt in the nested vector interrupt controller */
  /* TC4_IRQn where 4 is the timer number * timer channels (3) + the channel number (=(1*3)+1) for timer1 channel1 */
  NVIC_EnableIRQ(TC4_IRQn);

  // this is a cheat - enable the DAC
  analogWrite(DAC1,0);
 /* memset(testStr, 0x7E, sizeof(testStr));
  for(int i=0;i<100;i++){
    sendbitdata(testStr,256);
  }*/
  initGPS();
   digitalWrite(led, HIGH);
   //Temperature Sensor
   sensors.begin();
   if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Inside Device 0"); 
   if (!sensors.getAddress(outsideThermometer, 0)) Serial.println("Unable to find address for Outside Device 0"); 
   sensors.setResolution(insideThermometer, 9);
    sensors.setResolution(outsideThermometer, 9);
    initSPI();
}

void loop()
{

  takePicture();
  
 //$$PSB,sequence,time,lat,long,altitude,speed,satellites,lock,temp_in,temp_out,Vin*CHECKSUM\n 
 //   6      4      8   8    9     5       3        2       1     3        3     3 =56bytes
  
  //test=part2+getgpsTime()+separator+getlat()+separator+getlong()+separator+getalt()+separator+getspeed()+separator+getnumsatellites()+separator+getage()+part3;
 
  sendTelemetry();
 // Serial.println(gps.time.value());
  encodeImage(seqimg,callsign);
  delay(20000);
  seqimg++;
 // transmiton=1;
  //sendTelemetry();
 // memset(testStr, 0x7E, sizeof(testStr));
 // sendbitdata(testStr,256);
  //   delay(3000);
 
  
  // sendTelemetry();


}
void sendTelemetry(){
  memset(testStr, 0, sizeof(testStr));
  sensors.requestTemperatures();
  tempin = getTemperature(insideThermometer);
  tempout = getTemperature(outsideThermometer);
  voltage=getVoltage();
 // getGPSdata();
  starttimer();
  while(!getGPSdata()&&timerstarted);//||getage()!="1");//Wait for GPS data & add timer elapesed here.  
  stoptimer();
  test=callsign+seqnostr+getgpsTime()+getlat()+getlong()+getalt()+getspeed()+getnumsatellites()+getage()+tempin+tempout+voltage;
 // seqNumStr = String(seq++, DEC);
  seq++;
  sprintf(bufseq,"%04d:",seq);
  for(int i =0;i<4;i++){
    test[i+6]=bufseq[i];
  }
  if (seq==10000) seq=0;
  //Serial.println(seqNumStr);
 /* for(int i=0;i < seqNumStr.length();i++){
    test.setCharAt(12-i,seqNumStr.charAt(seqNumStr.length()-1-i));
  }
  //test.setCharAt(9,seqNumStr.charAt(0));
  // test.getBytes(testStr,test.length());
  for(int i=0;i<test.length();i++){
    testStr[i]=test[i];
  }
 */
 

  crc=rtty_CRC16_checksum(testStr,test.length());

  // crcbyte[0]=crc;
  // crcbyte[1]=(crc>>8);
  //Serial.println("CRC");
  // Serial.println(crc);
  crchex = String(crc,HEX);
  crchex.toUpperCase();
  if (crchex.length()<4){
    int ln = 4-crchex.length();
    for(int i=ln;i<4;i++){
      crcbyte[i]=crchex[i-ln];

    }
    for(int i=0;i<ln;i++){
      crcbyte[i]= 0x30;
    }
  }
  else{ 

    for(int i=0;i<4;i++){
      crcbyte[i]=crchex[i];
    }
  }


  //Serial.println(sizeof(encodeArray));
  //Serial.println(enc.length());

  // Serial.print(enc[i],HEX);


  //Serial.println(sizeof(enc));
  /*sendbitdata(flags,20);
   sendbitdata(flags,20);
   sendbitdata(flags,20);
   */
  memset(testStr, 0xAA, sizeof(testStr));
/*
  for(int i=0;i<test.length();i++){
    testStr[i+4]=test[i];                  //Create space for the $$$$
  }
  for(int i=0;i<4;i++){
    testStr[test.length()+i+4]=crcbyte[i];
  }
  for(int i=0;i<4;i++){
    testStr[i]=hdrbyte[i];
  }
  testStr[test.length()+8]=eol[0];*/
  testStr[0]=0x24;
  testStr[1]=0x24;
  for(int i=0;i<test.length();i++){
    testStr[i+2]=test[i];                 
  }
//  Serial.println(test);
  encode_rs_8(&testStr[0], &testStr[223], 0);
  // sendbitdata(flags,sizeof(flags));
  sendbitdata(flags,20);//Preamble
  sendbitdata(encodeArray,12);//Goldcode
  // sendbitdata(hdrbyte,4);
  shuffle(&testStr[0]);
  sendbitdata(testStr,256); //Data

  //sendbitdata(crcbyte,4);
  //sendbitdata(eol,1);
  //sendbitdata(high);
  //sendbitdata(low);
  sendbitdata(flags,5); //Tail
  /* sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,sizeof(flags));
   sendbitdata(flags,20);*/
   
   test=startHdr+callsign+separator+seqnostr+separator+getgpsTime()+separator+getlat()+separator+getlong()+separator+getalt();
   sprintf(bufseq,"%04d:",seq);
  for(int i =0;i<4;i++){
    test[i+14]=bufseq[i];
  }
   for(int i=0;i<test.length();i++){
       cwstring[i]=test[i];
    }
    send_morse=true;
  // sendMorse(cwstring,56);

}
void starttimer(){
  timerstarted=true;
}
void stoptimer(){
  timerstarted=false;
  timer=0;
}
int gettimerstatus(){
  return timerstarted?1:0; 
}
void sendSSDVpic(uint8_t *packet)
{
  sendbitdata(flags,50);//Was 50
  sendbitdata(encodeArray,12);
  shuffle(&packet[0]);
  sendbitdata(packet,256);
  sendbitdata(flags,5);

}
void sendbitdata(byte* toSend,int length)
{
  int bt = 0;
  byte inbyte;
  //Serial.println(toSend.length());
  //Serial.println(send_buffer.getSize());
  
  while(send_buffer.getSize()>9000){
    delay(2);
   // transmiton=1;
  }
  if(send_buffer.getSize()<512){
   // transmiton=0;
  }
  for (int i = 0; i < length; i++)
  {
    //  noInterrupts();
    send_buffer.put(toSend[i]);


    // interrupts();

    /*   inbyte = toSend[i];
     //String tstr = String(inbyte,HEX);
     // Serial.print(tstr);
     for (int k = 0; k < 8; k++)
     {                                               //do the following for each of the 8 bits in the byte
     bt = inbyte & 0x01;                           //strip off the rightmost bit of the byte to be sent (inbyte) 
     sendAX25tone1200BAUD(bt);
     inbyte = (byte)(inbyte >> 1);
     }*/
  }

}
void sendAX25tone1200BAUD(int bt){


  //Serial.println(bufcnt); 
  ulPhaseIncrement = toneRegister[bt];
  bufcnt=70;
  while(bufcnt!=0){
    ;
    // Serial.println(bufcnt);
  }
}

void TC4_Handler()//TC4_Handler()
{
  TC_GetStatus(TC1, 1);
  
  if(timerstarted){
    timer++;
    if (timer == timer5)
    {
      timerstarted=false;
      timer=0;
    }      
  }
if(send_morse){
  timer10ms++;
  if(timer10ms==960){
    if(sendMorse(cwstring,56)==1) send_morse=false;
    timer10ms=0;
  }
}
  if (bufcnt!=0){

    // We need to get the status to clear it and allow the interrupt to fire again


    ulPhaseAccumulator += ulPhaseIncrement;   // 32 bit phase increment, see below

    // if the phase accumulator over flows - we have been through one cycle at the current pitch,
    // now we need to reset the grains ready for our next cycle
    if(ulPhaseAccumulator >= SAMPLES_PER_CYCLE_FIXEDPOINT)
    {
      // DB 02/Jan/2012 - carry the remainder of the phase accumulator
      ulPhaseAccumulator -= SAMPLES_PER_CYCLE_FIXEDPOINT;
    }

    // get the current sample  
    uint32_t ulOutput = nSineTable[ulPhaseAccumulator>>20];

    // we cheated and user analogWrite to enable the dac, but here we want to be fast so
    // write directly 
    dacc_write_conversion_data(DACC_INTERFACE, ulOutput);
    bufcnt--;

  }
  if(bufcnt==0){
    if(K<8){
      bti = inbytei & 0x01;                           //strip off the rightmost bit of the byte to be sent (inbyte) 
      ulPhaseIncrement = toneRegister[bti];

      inbytei = (byte)(inbytei >> 1);
      K++;
      bufcnt=70;
    }
    else{
      K=8;
      if(send_buffer.getSize()> 0){             
        K=0;
        // noInterrupts();
        inbytei = send_buffer.get();
        //  Serial.print(send_buffer.getSize());
        //  Serial.print(" ");
        //  Serial.println(inbytei,HEX);
        // interrupts();
        bti = inbytei & 0x01;                           //strip off the rightmost bit of the byte to be sent (inbyte) 
        ulPhaseIncrement = toneRegister[bti];
        //
        inbytei = (byte)(inbytei >> 1);
        K++;    
        bufcnt=70;           
      }   
    }
  }
}
void TC4_Handler4800()
{
  TC_GetStatus(TC1, 1);
  if(transmiton==0) return;
  if (bufcnt!=0){

    //Serial.print(waveIndex);
    uint32_t ulOutput = baud_4800_table[waveIndex];
    waveIndex++;
    // we cheated and user analogWrite to enable the dac, but here we want to be fast so
    // write directly 
    dacc_write_conversion_data(DACC_INTERFACE, ulOutput);
    bufcnt--;

  }
  if(bufcnt==0){
    if(K<8){
      bti = inbytei & 0x01;                           //strip off the rightmost bit of the byte to be sent (inbyte) 
      selecttable();

      inbytei = (byte)(inbytei >> 1);
      K++;
     // waveIndex=0;
      bufcnt=20;

    }
    else{
      K=8;
      if(send_buffer.getSize()> 0){             
        K=0;
        // noInterrupts();
        inbytei = send_buffer.get();
        //  Serial.print(send_buffer.getSize());
        //  Serial.print(" ");
        //  Serial.println(inbytei,HEX);
        // interrupts();
        bti = inbytei & 0x01;                           //strip off the rightmost bit of the byte to be sent (inbyte) 
        selecttable();
        //
        inbytei = (byte)(inbytei >> 1);
        K++; 
     //   waveIndex=0;   
        bufcnt=20;           
      }   
    }
  }
}
void selecttable(){
  //  Serial.println(" ");
  txshreg <<= 1;
  if (bti == 1) txshreg |= 1;
  if ((txshreg & 0x20000) == 0x20000) txshreg ^= 0x0021;
  if ((txshreg & 0x40000) == 0x40000) bti = 1; else bti= 0; 
  if (previoussentbit == 1)
  {


    if (bti == 1)
    {
      //One to One Tra
      waveIndex=60;
    }
    else
    {
      waveIndex=20;
    }
  }
  else
  {
    if (bti == 1)
    {
      //Zero to 1
      waveIndex=00;
    }
    else
    {
      //Zero to Zero
      waveIndex=40;
    }
  }

  previoussentbit = bti;
}
String getTemperature(DeviceAddress deviceAddress)
{
 //char buf[16];
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit
 
  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  sprintf(tbuf,"%+03.0f",tempC);
  
  Serial.print("Temp C: ");
  Serial.print(tbuf);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
  return tbuf;
}

String getVoltage(){
  analogReadResolution(10);
  int sensorValue = analogRead(A0);
  double voltage= 2*sensorValue * (3.3 / 1024.0)*100;
  Serial.print("ADC 10-bit (default) : ");
  Serial.println(analogRead(A0));
  Serial.println(voltage);
  sprintf(tbuf,"%03.0f",voltage);
  Serial.println(tbuf);
  return tbuf;
}
void initSPI(){
  SPI.begin(10);
  SPI.setBitOrder(10,MSBFIRST) ;
  //Set Max Power
  SPI.transfer(10, 0xED, SPI_CONTINUE);
  SPI.transfer(10, 0x07);
  
  //Set to Unmodulated Carrier
  SPI.transfer(10, 0xF1, SPI_CONTINUE);
  SPI.transfer(10, 0x00);
  
 //Set Deviation
  SPI.transfer(10, 0xF2, SPI_CONTINUE);
  SPI.transfer(10, 0x38);
 
  //Set offet
  SPI.transfer(10, 0xF3, SPI_CONTINUE);
  SPI.transfer(10, 0x00);
  SPI.transfer(10, 0xF4, SPI_CONTINUE);
  SPI.transfer(10, 0x00);
  
 //Set no hopping
  SPI.transfer(10, 0xF9, SPI_CONTINUE);
  SPI.transfer(10, 0x00);
  SPI.transfer(10, 0xFA, SPI_CONTINUE);
  SPI.transfer(10, 0x00);
  
  //Set Frequency 434.650Mhz
  SPI.transfer(10, 0xF5, SPI_CONTINUE);
  SPI.transfer(10, 0x53);
  
  SPI.transfer(10, 0xF6, SPI_CONTINUE);
  SPI.transfer(10, 0x74);
  
  SPI.transfer(10, 0xF7, SPI_CONTINUE);
  SPI.transfer(10, 0x40);
  
  SPI.transfer(10, 0x87, SPI_CONTINUE);
  SPI.transfer(10, 0x09);
}



