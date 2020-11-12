// 29 Oct 2018
// sketch for use with Android App 'DCC Railway'using LMD18200 h-bridge
// updated to include XOR of data - now removed from App
// use digital pins 6 and 5 for DCC out

///////////////////////////////////////////////////////Variables from DCCSender
#define DEBUG true
#include <SoftwareSerial.h>
long t = 0;
long lastmillis = 0;
long interval = 1000;
//String inString;
int a[8];
int preamable_type = 0;
int Address;
//Timer frequency is 2MHz for ( /8 prescale from 16MHz )
#define TIMER_SHORT 0x8B  // 58usec pulse length 
//#define TIMER_LONG  0x3B  // 100usec pulse length
#define TIMER_LONG  0x17  // 100usec pulse length  

unsigned char last_timer = TIMER_SHORT; // store last timer value

unsigned char flag = 0; // used for short or long pulse
unsigned char every_second_isr = 0;  // pulse up or down
boolean print_ = true;
boolean new_pkt = true;


// definitions for state machine
#define PREAMBLE 0
#define SEPERATOR 1
#define SENDBYTE  2

#define BIG_PREAMBLE 24
#define SMALL_PREAMBLE 16
unsigned char state = PREAMBLE;
unsigned char preamble_count = 12;
unsigned char outbyte = 0;
unsigned char cbit = 0x80;


// variables

unsigned char  xdata = 0, data = 0, data_f = 0, data_f1 = 0, data_f2 = 0;

int locoAdr = 8;   // this is the default address of the loco
byte speed_ = 0;
bool direction_ = true;
#define FORWARD true
#define REVERSE false

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

int msgIndex = 1;
int byteIndex = 0;

#define LONG_WAIT 50;
#define SHORT_WAIT 26;



//////////////////////////////////////////////////////////Variables for Dcc Sniffer
byte refreshTime = 1; // Time between DCC packets buffer refreshes in s
byte packetBufferSize = 32; // DCC packets buffer size

#define TIMER_PRESCALER 64
#define DccBitTimerCount (F_CPU * 80L / TIMER_PRESCALER / 1000000L)
// 16000000 * 80 / 64 / 1000000 = 20; 20 x 4usecs = 80us

boolean packetEnd;
boolean preambleFound;

const byte bitBufSize = 50; // number of slots for bits
volatile byte bitBuffer[bitBufSize];
volatile byte bitBuffHead = 1;
volatile byte bitBuffTail = 0;

byte pktByteCount = 0;
byte packetBytesCount;
byte preambleOneCount;
byte dccPacket[6]; // buffer to hold a packet
byte instrByte1;
byte decoderType; //0=Loc, 1=Acc
byte bufferCounter = 0;
byte isDifferentPacket = 0;
byte showLoc = 0;


unsigned int decoderAddress;
unsigned int packetBuffer[64];
unsigned int packetNew = 0;

unsigned long timeToRefresh = millis() + refreshTime * 1000;

//Setup Timer2. (Used for Dcc Sender)
//Configures the 8-Bit Timer2 to generate an interrupt at the specified frequency.
//Returns the time load value which must be loaded into TCNT2 inside your ISR routine.
void SetupTimer2() {
  //Timer2 Settings: Timer Prescaler /8, mode 0
  //Timmer clock = 16MHz/8 = 2MHz oder 0.5usec
  TCCR2A = 0;
  TCCR2B = 0 << CS22 | 1 << CS21 | 0 << CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 = 1 << TOIE2;

  //load the timer for its first cycle
  TCNT2 = TIMER_SHORT;
}

//Timer2 overflow interrupt vector handler
ISR(TIMER2_OVF_vect) {

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
    latency = TCNT2;
    TCNT2 = latency + last_timer;
  }  else  {  // != every second interrupt, advance bit or state
    PORTD = B01000000;  //for LMD18200 digitalWrite(6, 1); digitalWrite(5, 0)
    //digitalWrite(6, 1); digitalWrite(5, 0);
    every_second_isr = 1;
    switch (state)  {
      case PREAMBLE:
        flag = 1; // short pulse
        if (preamble_count == SMALL_PREAMBLE) {
          if (new_pkt) {
            print_ = true;
            new_pkt = false;
            Serial.println("Starting preamble");
          }
        }
        if (preamble_count == 8 && print_) Serial.print(" ");
        if (print_) Serial.print(1);
        preamble_count--;
        if (preamble_count == 0)  {  // advance to next state
          if (print_) Serial.print(" ");
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
          if (print_) Serial.print(1);
          if (print_) Serial.print(" Done packet. Len: ");
          if (print_) Serial.println(msg[msgIndex].len);
          print_ = false;
          state = PREAMBLE;
          if (preamable_type == 0) {
            preamble_count = SMALL_PREAMBLE;    // normal preamble length of 16 '1's
          } else if (preamable_type == 1) {
            preamble_count = BIG_PREAMBLE;    // preamble of 24 '1's for CV1 write
          }
        } else {
          flag = 0; // long pulse
          if (print_) Serial.println(0);
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
          if (print_) Serial.print(1);
        }  else  {
          flag = 0;  // send long pulse
          if (print_) Serial.print(0);
        }
        cbit = cbit >> 1;
        if (cbit == 0)  {// send separtor and advance to next byte
          byteIndex++;
          if (print_) Serial.print(" ");
          state = SEPERATOR ;
        }
        break;
    }
    if (flag)  {  // if data==1 then short pulse
      latency = TCNT2;
      TCNT2 = latency + TIMER_SHORT;
      last_timer = TIMER_SHORT;

    }  else  {   // long pulse
      latency = TCNT2;
      TCNT2 = latency + TIMER_LONG;
      last_timer = TIMER_LONG;
    }
  }
}

//////////////////////////////////////////////////////////Functions from Dcc Sniffer
void getPacket() {
  preambleFound = false;
  packetEnd = false;
  packetBytesCount = 0;
  preambleOneCount = 0;
  while (! packetEnd) {
    if (preambleFound) getNextByte();
    else checkForPreamble();
  }
}

//========================

void checkForPreamble() {
  byte nextBit = getBit();
  if (nextBit == 1) preambleOneCount++;
  if (preambleOneCount < 10 && nextBit == 0) preambleOneCount = 0;
  if (preambleOneCount >= 10 && nextBit == 0) preambleFound = true;
}

//========================
void getNextByte() {
  byte newByte = 0;
  for (byte n = 0; n < 8; n++) newByte = (newByte << 1) + getBit();
  packetBytesCount ++;
  dccPacket[packetBytesCount] = newByte;
  dccPacket[0] = packetBytesCount;
  if (getBit() == 1) packetEnd = true;
}

//========================

byte getBit() {
  // gets the next bit from the bitBuffer
  // if the buffer is empty it will wait indefinitely for bits to arrive
  byte nbs = bitBuffHead;
  while (nbs == bitBuffHead) byte nbs = nextBitSlot(bitBuffTail); //Buffer empty
  bitBuffTail = nbs;
  return (bitBuffer[bitBuffTail]);
}

//========================

void beginBitDetection() {
  TCCR0A &= B11111100;
  attachInterrupt(0, startTimer, RISING);
}

//========================

void startTimer() {
  OCR0B = TCNT0 + DccBitTimerCount;
  TIMSK0 |= B00000100;
  TIFR0  |= B00000100;
}

//========================

ISR(TIMER0_COMPB_vect) {
  byte bitFound = ! ((PIND & B00000100) >> 2);
  TIMSK0 &= B11111011;
  byte nbs = nextBitSlot(bitBuffHead);
  if (nbs == bitBuffTail) return;
  else {
    bitBuffHead = nbs;
    bitBuffer[bitBuffHead] = bitFound;
  }
}

//========================

byte nextBitSlot(byte slot) {
  slot ++;
  if (slot >= bitBufSize) slot = 0;
  return (slot);
}

//========================

void printPacket() {
  Serial.print(" ");
  for (byte n = 1; n < pktByteCount; n++) {
    Serial.print(" ");
    Serial.print(dccPacket[n], BIN);
  }
  Serial.println(" ");
}

//========================

void refreshBuffer() {
  timeToRefresh = millis() + refreshTime * 1000;
  for (byte n = 0; n < packetBufferSize; n++) packetBuffer[n] = 0;
  bufferCounter = 0;
  Serial.println("-");

}

void setup() {
  Serial.begin(38400);
  analogReference(INTERNAL);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // turn bridge o/p off
  delay(500); // wait for bluetooth module to start
  //Serial.println("---");
  //Serial.println("DCC Packet Analyze started");
  //Serial.print("Updates every ");
  //Serial.print(refreshTime);
  //Serial.println(" seconds");
  //Serial.println("---");
  beginBitDetection();
  DDRD = B01100000;   //  register D5 for digital pin 5, D6 for digital pin 6
  clear_mem();
  SetupTimer2();
  t = millis();
  lastmillis = millis();
  digitalWrite(13, LOW); // turn bridge o/p on
}

void loop() {
  getPacket();
  byte speed;
  byte checksum = 0;

  if (millis() > timeToRefresh) refreshBuffer();
  pktByteCount = dccPacket[0];
  if (!pktByteCount) return; // No new packet available

  for (byte n = 1; n <= pktByteCount; n++) checksum ^= dccPacket[n];
  //checksum=0; //Comment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (checksum) return; // Invalid Checksum

  else { // There is a new packet with a correct checksum
    isDifferentPacket = 1;
    for (byte n = 0; n < packetBufferSize ; n++) { // Check if packet is not already in the buffer.
      // The checksum byte is used for the test, not ideal, some new packets may not show (only 256 different checksums)
      if (dccPacket[pktByteCount] == packetBuffer[n]) isDifferentPacket = 0;
    }

    if (isDifferentPacket) {  // packet does not yet exist in the packet buffer
      packetBuffer[bufferCounter] = dccPacket[pktByteCount]; // add new packet to the buffer
      bufferCounter = (++bufferCounter) & (packetBufferSize - 1);

      if (dccPacket[1] == B11111111) { //Idle packet
        //Serial.println("Idle ");
        return;
      }

      if (!bitRead(dccPacket[1], 7)) { //bit7=0 -> Loc Decoder Short Address
        decoderAddress = dccPacket[1];
        instrByte1 = dccPacket[2];
        decoderType = 0;
      }
      else {
        if (bitRead(dccPacket[1], 6)) { //bit7=1 AND bit6=1 -> Loc Decoder Long Address
          decoderAddress = 256 * (dccPacket[1] & B00000111) + dccPacket[2];
          instrByte1 = dccPacket[3];
          decoderType = 0;
        }
        else { //bit7=1 AND bit6=0 -> Accessory Decoder
          decoderAddress = dccPacket[1] & B00111111;
          instrByte1 = dccPacket[2];
          decoderType = 1;
        }
      }
      if (showLoc) {
        Serial.print("Loc ");
        Serial.print(decoderAddress);
      }

      byte instructionType = instrByte1 >> 5;
      switch (instructionType) {

        case 0:
          Serial.print(" Control ");
          break;


        case 1: // Advanced Operations
          if (instrByte1 == B00111111) { //128 speed steps
            if (bitRead(dccPacket[pktByteCount - 1], 7)) {
              direction_ = FORWARD;
            }
            else {
              direction_ = REVERSE;
            }
            byte speed = dccPacket[pktByteCount - 1] & B01111111;
            if (speed == 1) { //E-Stop
              clear_mem();
            }
            else {
              speed_ = speed / 4.5;
              set_velocity(speed_, direction_);
            }
          }
          else if (instrByte1 == B00111110) { //Speed Restriction
            if (bitRead(dccPacket[pktByteCount - 1], 7)) Serial.print(" On ");
            else Serial.print(" Off ");
            Serial.print(dccPacket[pktByteCount - 1])&B01111111;
          }
          break;
      }
    }
  }
}

void set_velocity(byte spd, bool dir) {
  Serial.print("Speed: ");
  Serial.println(speed_);

//  if (speed_ == 0) {
//    clear_mem();
//    return;
//  }

  speed_ = spd;
  direction_ = dir;
  build_speed_packet();
}

void clear_mem() {
  noInterrupts();
  msg[1].len = 3;
  msg[1].data[0] = 0;
  msg[1].data[1] = 0;
  msg[1].data[2] = (msg[1].data[0] ^ msg[1].data[1]);
  new_pkt = DEBUG;
  interrupts();
}

void build_speed_packet() {
  speed_++;
  byte byt = B01000000;
  if (direction_)
    byt += (1 << 5);
  byte packet = byt;
  packet += speed_;
  build_message(packet);
}

void build_message(byte speed_) {
  noInterrupts();
  msg[1].len = 3;
  msg[1].data[0] = 8;
  msg[1].data[1] = speed_;
  Serial.println(msg[1].data[1], BIN);
  msg[1].data[2] = (msg[1].data[0] ^ msg[1].data[1]);
  new_pkt = DEBUG;
  interrupts();
}
