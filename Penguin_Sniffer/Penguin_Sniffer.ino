///////////////////////////////////////////////////////
//
// DCC packet analyze: Ruud Boer, October 2015
// DCC packet capture: Robin McKay, March 2014
//
// The DCC signal is detected on Arduino digital pin 2
//
// Set the Serial Monitor Baud Rate to 38400 !!
//
// Keyboard commands that can be sent via Serial Monitor:
// 1 = 1s refresh time
// 2 = 2s 
// 3 = 4s (default)
// 4 = 8s
// 5 = 16s
// 6 = 4 DCC packet buffer
// 7 = 8
// 8 = 16
// 9 = 32 (default)
// 0 = 64
// a = show accessory packets toggle
// l = show locomotive packets toggle
//
////////////////////////////////////////////////////////

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

byte pktByteCount=0;
byte packetBytesCount;
byte preambleOneCount;
byte dccPacket[6]; // buffer to hold a packet
byte instrByte1;
byte decoderType; //0=Loc, 1=Acc
byte bufferCounter=0;
byte isDifferentPacket=0;
byte showLoc=1;
byte showAcc=1;


unsigned int decoderAddress;
unsigned int packetBuffer[64];
unsigned int packetNew=0;

unsigned long timeToRefresh = millis() + refreshTime*1000;

//========================

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
  return(slot);
}

//========================

void printPacket() {
  Serial.print(" ");
  for (byte n=1; n<pktByteCount; n++) {
    Serial.print(" ");
    Serial.print(dccPacket[n],BIN);
  }
  Serial.println(" ");
}

//========================

void refreshBuffer() {
  timeToRefresh = millis() + refreshTime*1000;
  for (byte n=0; n<packetBufferSize; n++) packetBuffer[n]=0;
  bufferCounter=0;
  Serial.println("-");

}

//========================

void setup() {
  Serial.begin(38400); // 38400 when on DCC, 9600 when testing on 123Circuits !!!!!!!!!!!!!!!!!!!!!!!
  Serial.println("---");
  Serial.println("DCC Packet Analyze started");
  Serial.print("Updates every ");
  Serial.print(refreshTime);
  Serial.println(" seconds");
  Serial.println("---");
  beginBitDetection(); //Uncomment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
}

//====================

void loop() {
  getPacket(); //Uncomment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  byte speed;
  int speed_;
  byte checksum = 0;
  
  if (millis() > timeToRefresh) refreshBuffer();
  pktByteCount = dccPacket[0];
  if (!pktByteCount) return; // No new packet available

  for (byte n = 1; n <= pktByteCount; n++) checksum ^= dccPacket[n];
  //checksum=0; //Comment this line when on DCC !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  if (checksum) return; // Invalid Checksum
  
  else { // There is a new packet with a correct checksum
    isDifferentPacket=1;
    for (byte n=0; n<packetBufferSize ; n++) {// Check if packet is not already in the buffer. 
    // The checksum byte is used for the test, not ideal, some new packets may not show (only 256 different checksums)
      if (dccPacket[pktByteCount]==packetBuffer[n]) isDifferentPacket=0; 
    }

    if (isDifferentPacket) {  // packet does not yet exist in the packet buffer
      packetBuffer[bufferCounter] = dccPacket[pktByteCount]; // add new packet to the buffer
      bufferCounter = (++bufferCounter)&(packetBufferSize-1);
      
      if (dccPacket[1]==B11111111) { //Idle packet
        Serial.println("Idle ");
        return;
      }
    
      if (!bitRead(dccPacket[1],7)) { //bit7=0 -> Loc Decoder Short Address
        decoderAddress = dccPacket[1];
        instrByte1 = dccPacket[2];
        decoderType = 0;
      }
      else {
        if (bitRead(dccPacket[1],6)) { //bit7=1 AND bit6=1 -> Loc Decoder Long Address
          decoderAddress = 256 * (dccPacket[1] & B00000111) + dccPacket[2];
          instrByte1 = dccPacket[3];
          decoderType = 0;
        }
        else { //bit7=1 AND bit6=0 -> Accessory Decoder
          decoderAddress = dccPacket[1]&B00111111;
          instrByte1 = dccPacket[2];
          decoderType = 1;
        }
      }
      if (showLoc) {
        Serial.print("Loc ");
        Serial.print(decoderAddress);
        byte instructionType = instrByte1>>5;
        switch (instructionType) {

          case 0:
            Serial.print(" Control ");
          break;

          
          case 1: // Advanced Operations
            if (instrByte1==B00111111) { //128 speed steps
              if (bitRead(dccPacket[pktByteCount-1],7)) Serial.print(" Forw128 ");
              else Serial.print(" Rev128 ");
              byte speed = dccPacket[pktByteCount-1]&B01111111;
              if (!speed) Serial.print(" Stop ");
              else if (speed==1) Serial.print(" E-stop ");
              else {
                speed = speed / 4.5;
                speed_ = speed;
                Serial.print(speed);
                }
            }
            else if (instrByte1==B00111110) { //Speed Restriction
            if (bitRead(dccPacket[pktByteCount-1],7)) Serial.print(" On ");
              else Serial.print(" Off ");
              Serial.print(dccPacket[pktByteCount-1])&B01111111;
            }
          break;
        }
      Serial.print("\n");
      Serial.print(speed_);
      Serial.print("\n");
      }
    }
  }
}

//=====================
