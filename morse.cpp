/*This has been modified from this original morse code git entry*/
/*https://github.com/lilspikey/arduino_sketches/blob/master/morse/morse.ino*/
/*Inclusions included extended ascii table & interface onto SPI RFM43 device by VK3TBC 12/2/2013*/

#include <stdio.h>
#include <arduino.h>
#include <stdint.h>
#include <SPI.h>
extern void ddsUP();
extern void ddsDOWN();
#define LED_PIN 13
#define BUZZER_PIN 4

#define BUZZER_FREQUENCY 700

#define STATE_IDLE 0
#define STATE_DOT 1
#define STATE_DASH 2
#define STATE_SPACE 3
#define STATE_GAP 4

#define GAP_LENGTH 50
#define DOT_LENGTH 50
#define DASH_LENGTH 150
#define SPACE_LENGTH 150

#define MESSAGE_MAX 512

char* morse_table[] = {
  ".-", /* A */
  "-...", /* B */
  "-.-.", /* C */
  "-..", /* D */
  ".", /* E */
  "..-.", /* F */
  "--.", /* G */
  "....", /* H */
  "..", /* I */
  ".---", /* J */
  "-.-", /* K */
  ".-..", /* L */
  "--", /* M */
  "-.", /* N */
  "---", /* O */
  ".--.", /* P */
  "--.-", /* Q */
  ".-.", /* R */
  "...", /* S */
  "-", /* T */
  "..-", /* U */
  "...-", /* V */
  ".--", /* W */
  "-..-", /* X */
  "-.--", /* Y */
  "--..", /* Z */

  "-----", /* 0 */
  ".----", /* 1 */
  "..---", /* 2 */
  "...--", /* 3 */
  "....-", /* 4 */
  ".....", /* 5 */
  "-....", /* 6 */
  "--...", /* 7 */
  "---..", /* 8 */
  "----.", /* 9 */
  "..--..",    /*?  36*/
  "--..--",    /*,  37*/
  "---...",/*:  38*/
  ".-.-.-",   /*.   39*/
  ".--.-.",    /*$-->@ 40*/
  "-....-",    /*- 41*/
 " ", 
};

struct state {
  long timer;
  int state;
};

int seq_num = 0;
char *sequence = NULL;
struct state morse;

int char_num = 0;
int message_len = 0;
char message[MESSAGE_MAX];

void change_state(struct state *pstate, int new_state) {
  pstate->timer = millis();
  pstate->state = new_state;
}
//Send morse to DDS chip on SPI BUS
void sendSPI(double frequency){
  // Arduino Code
int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850
  for (int b=0; b<4; b++, freq>>=8) {
     SPI.transfer(10,(freq&0xFF), SPI_CONTINUE);
  }
  SPI.transfer(10,0x00);  // Final control byte, all 0 for 9850 chip
}
void morse_pulse_on() {
  digitalWrite(LED_PIN, HIGH);
   sendSPI(10100000);
  //SPI.transfer(10, 0x87, SPI_CONTINUE);
  //SPI.transfer(10, 0x09);
 // ddsUP();

  //tone(BUZZER_PIN, BUZZER_FREQUENCY); 
  // Serial.print("High ");
}

void morse_pulse_off() {
  digitalWrite(LED_PIN, LOW);
   sendSPI(0);
 // SPI.transfer(10, 0x87, SPI_CONTINUE);
 // SPI.transfer(10, 0x01);
   //ddsDOWN();
  //  Serial.print("LOW ");
  //noTone(BUZZER_PIN);
}

void run_morse() {
  long current_time = millis();
  long state_duration = (current_time - morse.timer);
  int next_state = morse.state;
  // Serial.print(morse.state);
  // Serial.print(" ");
  // Serial.println(state_duration);
  switch(morse.state) {
  case STATE_DOT:
    if ( state_duration < DOT_LENGTH ) {
      morse_pulse_on();
    }
    else {
      next_state = STATE_GAP;
    }
    break;
  case STATE_DASH:
    if ( state_duration < DASH_LENGTH ) {
      morse_pulse_on();
    }
    else {
      next_state = STATE_GAP;
    }
    break;
  case STATE_SPACE:
    if ( state_duration < SPACE_LENGTH ) {
      morse_pulse_off();
    }
    else {
      next_state = STATE_GAP;
    }
    break;
  case STATE_GAP:
    if ( state_duration < GAP_LENGTH ) {
      morse_pulse_off();
    }
    else {
      next_state = STATE_IDLE;
    }
  }

  if ( morse.state != next_state ) {
    change_state(&morse, next_state);
  }

  if ( morse.state == STATE_IDLE ) {
    if ( sequence && seq_num < strlen(sequence) ) {
      char next = sequence[seq_num];
      seq_num++;
      switch(next) {
      case '.':
        change_state(&morse, STATE_DOT);
        break;
      case '-':
        change_state(&morse, STATE_DASH);
        break;
      default:
        change_state(&morse, STATE_SPACE);
      }
    }
    else {
      if ( char_num < message_len ) {
        int offset = -1;
        char message_char = message[char_num];
        if ( 'a' <= message_char && message_char <= 'z' ) {
          offset = (int)(message_char - 'a');
        }
        else if ( 'A' <= message_char && message_char <= 'Z' ) {
          offset = (int)(message_char - 'A');
        }
        else if ( '0' <= message_char && message_char <= '9' ) {
          offset = (int)(message_char - '0') + 26;
        }

        if(message_char == '?') {
          offset=36;
        }
        if(message_char == ',') {
          offset=37;
        }
        if(message_char == ':') {
          offset=38;
        }
        if(message_char == '.') {
          // Serial.println("Dot");

          offset=39;
          // Serial.println(sequence = morse_table[offset]);
          //  Serial.println(strlen(sequence));
        }
        if(message_char == '$') {
          offset=40;
        }
        if(message_char == '-') {
          offset=41;
        }
          if(message_char == ' ') {
          offset=42;
        }
        if ( offset >= 0 ) {
          sequence = morse_table[offset];
          seq_num = 0;
        }
        char_num++;
        change_state(&morse, STATE_SPACE);
      }
      else {
        sequence = NULL;
      }
    }
  }
}

void initMorse() {
  pinMode(LED_PIN, OUTPUT);
  //  pinMode(BUZZER_PIN, OUTPUT);
  //  Serial.begin(9600);
}

int sendMorse(char *msg,int Length) {

  if(char_num==0){
    for(int i=0;i<Length;i++){
      message[i]=msg[i];      
    }
    // Serial.println("Initiail");
    message_len=Length;//56
  }
  if(char_num >= message_len){  
    morse.state == STATE_IDLE; 
    char_num = 0;
    message_len = 0;
    // Serial.println("done");
    return 1;
  }
  //Serial.println("running");
  run_morse(); 
  return 0; 
}






