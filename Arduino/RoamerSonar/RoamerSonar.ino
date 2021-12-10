//from https://forum.arduino.cc/index.php?topic=494594.msg3374836#msg3374836
#define TriggerPin  2                  // output pin all trigger pins are linked to.
#define SamplesToAverage 0             // number of ping samples to average together for use
#define DelayBetweenPings 15            // delay after ping data is completely recieved all have reported in and the next trigger pulse in miliseconds (5 seems to be great)

unsigned long Timer, zTimer, xTimer; // Delay timers
volatile unsigned long SampleAge[20];
volatile int PinArray[20] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};            // List of pins that could be used as ping inputs:
unsigned long PingTime[20] ;
volatile int ToCompleteCtr;
volatile  unsigned long PingTimeX[20];
volatile  int PingSamplesX[20];
volatile  unsigned long edgeTime[20];
volatile  uint8_t PCintLast[3];
volatile  uint8_t mask[3];
int PinMask[3];
float Measurements[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte *MeasurementsB;

bool SerialEvent = false;                   // Detects serial imput  1
int I2CsendDataGo = 0;
uint8_t dataB[32] ;
int c, Pin;
int x = 0;
float inches, cm;
int DegSpread = 360;                  
static int SensorCount = 0;
volatile byte PinList[25];            

void setup() {

  Serial.begin(115200);
  //Serial.println("Send 1 for Readable Data, Send 0 For Interger Data");
  SensorCount = 0;
  pinMode(TriggerPin, OUTPUT);
  // enable interrupt for pin...
  // Select the pins you want to use for input
  // pciSetup(0); // Serial Communication
  // pciSetup(1); // Serial Communication
  // pciSetup(2); //This is my trigger pin
  pciSetup(3); //0 degrees (assuming 4 sensors)
  pciSetup(4); //90 degrees (assuming 4 sensors)
  pciSetup(5); //180 degrees (assuming 4 sensors)
  pciSetup(6); //270 degrees (assuming 4 sensors)
  //pciSetup(7); //more sensors one day?
  //pciSetup(8); //more sensors one day?
  //pciSetup(9); //more sensors one day?
 // pciSetup(10); //more sensors one day?
 // pciSetup(11); // SPI communications
 // pciSetup(12); // SPI communications
 // pciSetup(13); // SPI communications
 // pciSetup(14); // A0
 // pciSetup(15); // A1
 // pciSetup(16); // A2
 // pciSetup(17); // A3
 // pciSetup(18); // A4  i2c Communication
 // pciSetup(19); // A5  i2c Communication
 // pciSetup(20); // A6  for some clones
 // pciSetup(21); // A7  for some clones

  //Serial.println("SensorCount:" + (String)SensorCount);
}

void loop() {
  Timer = millis(); // timing events!
  PingIt(); // Manage ping data
  RadarOut2(DegSpread);
  delay(600);
}

void RadarOut2(int Degrees) {
  static int x = 0;
  static bool xz = true;
  static byte Sensor = 0;
  int deg = Degrees / SensorCount;
  Sensor = 0;
  Serial.print("SONAR{");
  while (Sensor < SensorCount) {
    if (Degrees > 0) {
      Serial.print((Sensor * deg) );
      Serial.print(":");
      Serial.print((int)Measurements[Sensor]);
      if (Sensor < SensorCount-1){
        Serial.print(",");
      }
      Sensor++;
    }
  }
  Serial.println("}");
}

void PingTrigger(int Pin) {
  digitalWrite(Pin, LOW);
  delayMicroseconds(1);
  digitalWrite(Pin, HIGH); // Trigger another pulse
  delayMicroseconds(5);
  digitalWrite(Pin, LOW);
}

bool AllClear() {
  return (!(PinMask[0] & PIND) && !(PinMask[1] & PINB) && !(PinMask[2] & PINC) && !ToCompleteCtr); //  all the input pins are LOW
}

void PingIt() {
  static int SampleCt = 0;
  static bool DelayLatch = true;
  if ( AllClear()) { // Wait
    if (DelayLatch) {
      xTimer = Timer + DelayBetweenPings;
      DelayLatch = false;
    }
    if ((Timer - xTimer) >= 0 ) {
      for (int i = 0; i < 20; i++) {
      if (SampleCt >= SamplesToAverage) {
        //Serial.println("T");
        byte Sensor = 0; //????????
        
          if ((PinArray[i] != -1) ) {
            if (PingSamplesX[i] > 0) {
              PingTime[i] =  (unsigned long) (PingTimeX[i] / PingSamplesX[i]); // average
              byte Sensor = findPin(i);
              Measurements[Sensor] = (float) (microsecondsToCentimeters(PingTime[i]));
            }
            PingTimeX[i] = 0;
            PingSamplesX[i] = 0;
            Sensor++; //???????
          }
          else{
          //Serial.println("Skipping sensor: " + (String)i);
          }
        }
        SampleCt = 0;
        SampleAge[i] = micros();
      }

      DelayLatch = true;
      SampleCt++;
      PingTrigger(TriggerPin); // Send another ping
      zTimer = Timer;
    }
  }
}

float microsecondsToCentimeters(long microseconds)
{
  return (float)microseconds / 29 / 2;
}

uint8_t DataMessage() {
  int x = 0;
  for (int i = 0; i < 15; i++ ) {
    dataB[x] = (uint8_t)(((uint8_t)Measurements[i]) >> 8);
    x++;
    dataB[x] = (uint8_t)Measurements[i];
    x++;
  }
}

// port change Interrupt
ISR(PCINT0_vect) { //this ISR pins 8~13
  static uint8_t pin;
  static unsigned long cTime;
  cTime = micros();         // micros() return a uint32_t
  pin = PINB; //  get the state of all pins bit 0 = pin 8, bit 1 = pin 9 dtc.
  ToCompleteCtr++;
  sei();                    // re enable other interrupts
  CheckTimers(8, 13,  1, pin, cTime);
}

ISR(PCINT1_vect) { //this ISR s A0~A5
  static uint8_t pin;
  static unsigned long cTime;
  cTime = micros();         // micros() return a uint32_t
  pin = PINC; //  get the state of all pins bit 0 = pin 8, bit 1 = pin 9 dtc.
  ToCompleteCtr++;
  sei();                    // re enable other interrupts
  CheckTimers(14, 19,  2, pin, cTime);
}

ISR(PCINT2_vect) { //this ISR pins 0-7
  static uint8_t pin;
  static unsigned long cTime;
  cTime = micros();         // micros() return a uint32_t
  pin = PIND; //  get the state of all pins bit 0 = pin 8, bit 1 = pin 9 dtc.
  ToCompleteCtr++;
  sei();                    // re enable other interrupts
  CheckTimers(0, 7,  0, pin, cTime);
}

void CheckTimers(uint8_t StartPin, uint8_t EndPin, uint8_t group, uint8_t  pin, unsigned long cTime )
{
  volatile uint16_t dTime;
  mask[group] = pin ^ PCintLast[group];
  PCintLast[group] = pin;        
  for (uint8_t ii = 0; ii <= (EndPin - StartPin); ii++) {
    if (mask[group] >> ii & 1) { // pin has changed
      if ((pin >> ii & 1))edgeTime[(ii + StartPin)] = cTime;
      else { // Pulse Went low calculate the duratoin
        dTime = cTime - edgeTime[(ii + StartPin)]; // Calculate the change in time
        PingTimeX[(ii + StartPin)] = PingTimeX[(ii + StartPin)] + dTime;
        PingSamplesX[(ii + StartPin)]++;
      }
    }
  }
  ToCompleteCtr--;  //when all interupts are complete this will return to zero
}

// Install Pin change interrupt for a pin, can be called multiple times
void pciSetup(byte pin)
{
  //Serial.println("Setting up pin:" + (String)pin);
  if (pin <= 7)PinMask[0] =  bitWrite(PinMask[0], pin, 1); // PIND for pins 0~7
  else if (pin > 13) PinMask[2] =  bitWrite(PinMask[2] , pin - 14, 1); // PINC for A0~A5 Starts on Pin 14
  else PinMask[1] =  bitWrite(PinMask[1] , pin - 8, 1); // PINB for pins 8~13
  pinMode(pin, INPUT);// enable interrupt for pin...
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
  //Serial.println("Setting flag pin:" + (String)pin);
  PinArray[pin] = 1;
  //Serial.println("done Setting flag pin:" + (String)pin);
  PinList[SensorCount] = pin;
  SensorCount++;
  }

void serialEvent() {
  SerialEvent = true;
}

int findPin(int i){
  for (int p=0; p<20; p++) {
     if (i == PinList[p]) {
       return p;
       break;
     }
  }
}
