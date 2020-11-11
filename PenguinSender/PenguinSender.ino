// 29 Oct 2018
// sketch for use with Android App 'DCC Railway'using LMD18200 h-bridge
// updated to include XOR of data - now removed from App

#include <SoftwareSerial.h>
//SoftwareSerial bluetooth(8,9);  // RX TX
int C;
int sensorValue;
long t = 0;
long lastmillis = 0;
long interval = 1000;
//String inString;
int a[8];
int preamable_type = 0;
int Address;



// use digital pins 6 and 5 for DCC out

//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x00  // 58usec pulse length 
//#define TIMER_LONG  0x3B  // 100usec pulse length  
#define TIMER_LONG  0x30  // 100usec pulse length  

unsigned char last_timer=TIMER_SHORT;  // store last timer value
   
unsigned char flag=0;  // used for short or long pulse
unsigned char every_second_isr = 0;  // pulse up or down
boolean print_ = true;
boolean new_pkt = true;


// definitions for state machine 
#define PREAMBLE 0    
#define SEPERATOR 1
#define SENDBYTE  2


unsigned char state= PREAMBLE;
unsigned char preamble_count = 12;
unsigned char outbyte = 0;
unsigned char cbit = 0x80;


// variables

unsigned char  xdata = 0, data = 0, data_f = 0,data_f1 = 0,data_f2 = 0;

int locoAdr = 8;   // this is the default address of the loco

// buffer for command
struct Message {
   unsigned char data[7];
   unsigned char len;
} ;

#define MAXMSG 3
// for the time being, use only 2 messages - the idle msg, the loco Speed msg, function msg

struct Message msg[MAXMSG] = { 
    { { 0xFF, 0, 0xFF, 0, 0, 0, 0}, 3},   // idle msg
    { { locoAdr, 0, 0,  0, 0, 0, 0}, 3}   // locoMsg with 128 speed steps 0x3f
  };               // loco msg must be filled later with speed and XOR data byte
                                
int msgIndex=1;  
int byteIndex=0;

#define LONG_WAIT 50;
#define SHORT_WAIT 26;

int trig_goal;
int trig_prog;

//Setup Timer2.
//Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
//Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
/*
void SetupTimer2(){
  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timmer clock = 16MHz/8 = 2MHz oder 0.5usec
  TCCR2A = 0;
  TCCR2B = 0<<CS22 | 1<<CS21 | 0<<CS20; 

  //Timer2 Overflow Interrupt Enable   
  TIMSK2 = 1<<TOIE2;

  //load the timer for its first cycle
  TCNT2=TIMER_SHORT; 
}
*/


void SetupTimer2(){
  cli();//stop interrupts
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 3;// = (16*10^6) / (5000000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  trig_prog = 0;
  trig_goal = SHORT_WAIT;
  sei();//allow interrupts
}


//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {
  if(trig_prog < trig_goal){
    trig_prog++;
    break;
  }
  
  //Capture the current timer value TCTN2. This is how much error we have
  //due to interrupt latency and the work in this function
  //Reload the timer and correct for latency.  
  // for more info, see http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
  unsigned char latency;
  // for every second interupt just toggle signal
  if (every_second_isr)  {
    PORTD = B01100000;  //use this instead of digitalWrite(6,1); digitalWrite(5,1);for LMD18200 ---  5 to 'DIR' DCC signal and 6 to 'PWM' held HIGH
    //digitalWrite(6,1); digitalWrite(5,1);
     every_second_isr = 0;        
     // set timer to last value
     latency=TCNT2;
     TCNT2=latency+last_timer;  
  }  else  {  // != every second interrupt, advance bit or state
     PORTD = B01000000;  //for LMD18200 digitalWrite(6, 1); digitalWrite(5, 0)
     //digitalWrite(6, 1); digitalWrite(5, 0);
     every_second_isr = 1; 
     switch(state)  {
       case PREAMBLE:
           flag=0; // short pulse
           if(preamble_count == 12){
             if(new_pkt){
              print_ = true;
              new_pkt = false;
              Serial.println("Starting preamble");
             }
           }
           if(preamble_count == 8 && print_) Serial.print(" ");
           if(print_) Serial.print(1);
           preamble_count--;
           if (preamble_count == 0)  {  // advance to next state
              if(print_) Serial.print(" ");
              state = SEPERATOR;
              // get next message
              //msgIndex++;
              //if (msgIndex >= MAXMSG)  {  msgIndex = 0; }  
              byteIndex = 0; //start msg with byte 0
           }
           break;
        case SEPERATOR:
           if (byteIndex >= msg[msgIndex].len)  {
            flag = 1;
            if(print_) Serial.print(1);
            if(print_) Serial.print(" Done packet. Len: ");
            if(print_) Serial.println(msg[msgIndex].len);
            print_ = false;
            state = PREAMBLE;
            if (preamable_type == 0){
              preamble_count = 12;    // normal preamble length of 16 '1's
            }else if (preamable_type == 1){
              preamble_count = 24;    // preamble of 24 '1's for CV1 write
            }
           }else{
            flag=0; // long pulse
            if(print_) Serial.println(0);
            // then advance to next state
            state = SENDBYTE;
            // goto next byte ...         
            outbyte = msg[msgIndex].data[byteIndex];
           }
           cbit = 0x80;  // send this bit next time first
           break;
        case SENDBYTE:
           if (outbyte & cbit)  { 
              flag = 1;  // send short pulse
              if(print_) Serial.print(1);
           }  else  {
              flag = 0;  // send long pulse
              if(print_) Serial.print(0);
           }
           cbit = cbit >> 1;
           if (cbit == 0)  {// send separtor and advance to next byte
            byteIndex++;
            if(print_) Serial.print(" ");
            state = SEPERATOR ;
           }
           break;
     }   
     if (flag)  {  // if data==1 then short pulse
        latency=TCNT2;
        TCNT2=latency+TIMER_SHORT;
        last_timer=TIMER_SHORT;
        
     }  else  {   // long pulse
        latency=TCNT2;
        TCNT2=latency+TIMER_LONG; 
        last_timer=TIMER_LONG;
     }  
  }
}

/*
ISR(TIMER2_COMPA_vect){
  if(trig_prog == trig_at){
    Serial.println(TCNT2);
    trig_prog == 0;
  }else trig_prog++;
}
*/

void setup(){
  Serial.begin(115200);
  analogReference(INTERNAL);
  pinMode(13, OUTPUT);   
  digitalWrite(13, HIGH); // turn bridge o/p off
  delay(500); // wait for bluetooth module to start
  Serial.println("Bluetooth Started");

  DDRD = B01100000;   //  register D5 for digital pin 5, D6 for digital pin 6
  clear_mem();
  //hardcode(B00000110);
 //Start the timer 
  SetupTimer2();
  t = millis();
  lastmillis = millis();
  digitalWrite(13, LOW); // turn bridge o/p on

  
}

void current(){
int i;
  int value = 0;
  int numReadings = 5;
for (i = 0; i < numReadings; i++){
    // Read sensor data.
    value = value + analogRead(A0);
    // 1ms pause adds more stability between reads.
    delay(1);
  }

  sensorValue = value/numReadings;
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 1.1V) = 1.08mv per division
  // 0.1 ohm resistor on current sense gives 200mv at 2 Amps or, 100mv per Amp
  // 1.08 mv per div for Internal Ref of 1.1v  : 100/1.08 = 92.6 divisions per 1000mA or 1 div = 10.8mA
  C = 10.8 * sensorValue ;  // mA
 
  if(C >2000){  // 2 amps
   Serial.println("Over Current");
   digitalWrite(6, LOW);  // logic control of DCC signal to low (off)
   digitalWrite(13, HIGH); // to brake, shorts out h-bridge o/p
  }
}

void loop() { 
  if (Serial.available()) {
    switch (Serial.read()) {
      case 49: 
        Serial.println("starting");
        hardcode(B01100100);
      break;
      case 48:
        Serial.println("stopping");
        clear_mem();
        //hardcode(B01100010);
      break;
    }
  }

}

void clear_mem(){
  noInterrupts();
  msg[1].len = 3;
  msg[1].data[0] = 0;
  msg[1].data[1] = 0;
  msg[1].data[2] = (msg[1].data[0] ^ msg[1].data[1]);
  new_pkt = true;
  interrupts();
}

void hardcode(int speed_){
  noInterrupts();
  msg[1].len = 3;
  msg[1].data[0] = 8;
  msg[1].data[1] = speed_;
  msg[1].data[2] = (msg[1].data[0] ^ msg[1].data[1]);
  new_pkt = true;
  interrupts();
}
  /*
 void amend_len4 (struct Message & x) 
{ 
 x.len = 4;
   //Serial.println(x.len);
}

void assemble_4_byte() { 
   noInterrupts(); 
   msg[1].data[0] = a[1] >> 8;
   msg[1].data[1] = a[1];
   msg[1].data[2] = a[2];
   msg[1].data[3] = (a[1] ^ (a[1] >> 8)^ a[2]);
   interrupts();
}

void amend_len3 (struct Message & x) 
{ 
 x.len = 3;
  //Serial.println(x.len);
}

void assemble_3_byte() { 
   noInterrupts(); 
   msg[1].data[0] = a[1];
   msg[1].data[1] = a[2];
   msg[1].data[2] = (a[1] ^ a[2]);
   msg[1].data[3] = 0;
   Serial.println(msg[1].data[0]);
   interrupts();
   
}


void print_data(){
 Serial.print(msg[1].data[0], DEC);
 Serial.print(",");
 Serial.print(msg[1].data[1], DEC);
 Serial.print(",");
 Serial.print(msg[1].data[2], DEC);
 Serial.print(",");
 Serial.print(msg[1].data[3], DEC);
 Serial.println(",");
  }
  */
