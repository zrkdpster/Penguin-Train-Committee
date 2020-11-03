int outPin = 4; //The output pin does not need to be a pwm pin if we are manualy varying the voltage


void zerozzz() {
  digitalWrite(outPin, HIGH); // sets the pin on
  delayMicroseconds(100);      // pauses for 100 microseconds
  digitalWrite(outPin, LOW); // sets the pin off
  delayMicroseconds(100);      // pauses for 100 microseconds
  //Serial.println("zero");
}

void onezzz() {
  digitalWrite(outPin, HIGH); // sets the pin on
  delayMicroseconds(58);      // pauses for 58 microseconds
  digitalWrite(outPin, LOW); // sets the pin off
  delayMicroseconds(58);      // pauses for 58 microseconds
  //Serial.println("one");
}


void setup() {
  Serial.begin(9600); //Begin serial monitor at 9600 baud
  pinMode(outPin, OUTPUT); //
  digitalWrite(outPin, LOW); // sets the pin off

  int resetPacket[]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
  int resetPacketLength = 44; //Set this to the number of elements in the corisponding packet for propper functionality in the loop
  
}



void loop() {

for (byte i = 0; i < resetPacketLength; i = i + 1) {
  if (resetPacket[i] == 0){
    zerozzz();
  } else {
    onezzz();
  }
}
 Serial.println("Locomotive reset");
  }
