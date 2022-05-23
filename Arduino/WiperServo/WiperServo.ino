//input: -100% to 100% of steering range over serial
//todo: detect failure to control
//todo: detect motor vin
//todo: lockout if values are out of safe range

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary
#include <Smoothed.h> // from https://www.arduino.cc/reference/en/libraries/smoothed/ //works in ide 1.8.19 //doesn't work properly with signed ints, apply on raw adc only.

#define SteeringWheelPin A0 //smoothed
#define SteeringFeedbackPin A1 //smoothed
#define LeftPedalPin A2 //smoothed
#define RightPedalPin A3 //not required
#define DividerPin A4
#define CurrentSensePin A5 //probably needs smoothing
#define BrakeHallPin A6 //probably needs smoothing

#define LocRemSwPin 5
#define DriveSwPin 6
#define RevSwPin 7
#define EstopPin 8
#define MotorEnPin 11
#define LPWMpin 10
#define RPWMpin 9
#define AUX1pin 3 //FET 12V OP only
#define AUX2pin 2 //FET 12V OP only
#define AUX3pin 4 //input only with 10k pulldown

#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2, 4);       // RX, TX

// Global variables for hoverboard
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  int16_t  brake;
  int16_t  driveMode;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

unsigned long previousMillisA = 0;
int deadband = 1;
int SteerCentreOffset = -100;
int PedalCentre = 550;
float revspd = 0.3;
int poslimit = 150; //150 is somewhat arbritrary number that represents 90 degrees each way
float izerooffset = 350.0; //adc counts at 0 amps
float ical = 30.0;//adc counts per 1amp
double ilim = 80; //in 0.1A
unsigned long CC_iterations = 0; //number of iterations in constant current mode
int lockout_time = 10; // time in constant current before tripping to lockout mode
bool lockout = false;
Smoothed <int> SteeringWheelVal;
Smoothed <int> SteeringFeedbackVal;
Smoothed <int> AccelPedalVal;
bool manualBraking;
bool currentLimiting;
int brkcmd;
int drvcmd;

// PID for posn control
double Pk1 = 7;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;
double Setpoint1, Input1, Output1;
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);

// PID current limit
double Pk2 = 0;  //I lim
double Ik2 = 15;
double Dk2 = 0;
double Input2, Output2;
PID PID2(&Input2, &Output2, &ilim, Pk2, Ik2 , Dk2, DIRECT);

double pwm;
volatile boolean done;
unsigned long start;

int pot;
int sp;
int pos = 0; //mapped steeringwheelval avg
int posraw = 0;
int I_reading = 0;

int pedalval = 0;
int pedaldeadband = 50;

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 50;        // time constant for timers

#define SPD_MODE        2               // [-] SPEED mode
#define TRQ_MODE        3               // [-] TORQUE mode

int16_t currentDriveMode = TRQ_MODE;

void setup() {
  pinMode(LocRemSwPin, INPUT);
  pinMode(DriveSwPin, INPUT);
  pinMode(RevSwPin, INPUT);
  pinMode(EstopPin, OUTPUT);
  pinMode(MotorEnPin, OUTPUT);
  pinMode(LPWMpin, OUTPUT);
  pinMode(RPWMpin, OUTPUT);
  pinMode(AUX1pin, OUTPUT);
  pinMode(AUX2pin, OUTPUT);
  //pinMode(AUX3pin, INPUT); //used for sw serial

  pinMode(SteeringWheelPin, INPUT);
  pinMode(SteeringFeedbackPin, INPUT);
  pinMode(LeftPedalPin, INPUT);
  pinMode(RightPedalPin, INPUT);
  pinMode(DividerPin, INPUT);
  pinMode(CurrentSensePin, INPUT);
  pinMode(BrakeHallPin, INPUT);

  SteeringWheelVal.begin(SMOOTHED_AVERAGE, 10);
  SteeringFeedbackVal.begin(SMOOTHED_AVERAGE, 10);
  AccelPedalVal.begin(SMOOTHED_AVERAGE, 10);

  setPwmFrequency(LPWMpin, 1); //62500hz?
  setPwmFrequency(RPWMpin, 1); //62500hz?
  Serial.begin(115200); //hardware serial to pi
  HoverSerial.begin(HOVER_SERIAL_BAUD); //software serial to hoverboard

  PID1.SetMode(AUTOMATIC);              // PID posn control loop
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);

  PID2.SetMode(AUTOMATIC);              // PID constant current loop
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(20);

  digitalWrite(MotorEnPin, HIGH);
}

void Send(int16_t uSteer, int16_t uSpeed, int16_t brake, int16_t driveMode)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.brake    = (int16_t)brake;
  Command.driveMode = (int16_t)driveMode;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed ^ Command.brake ^ Command.driveMode);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

void wiperServo(int sp) {
  Setpoint1 = constrain(map(sp, -100, 100, 0 - poslimit, poslimit), 0 - poslimit, poslimit);
  pot = SteeringFeedbackVal.get();//analogRead(SteeringFeedbackPin) + SteerCentreOffset;
  //Serial.print(" pos_sp:");
  //Serial.print (Setpoint1);
  //Serial.print(" pos_ip:");
  Input1 = map(pot, 0, 1023, -255, 255);
  //Serial.print (Input1);
  PID1.Compute();
  //Serial.print(" pos_op:");
  //Serial.print(Output1);
  I_reading = analogRead(CurrentSensePin);
  //Serial.print(" I_raw:");
  //Serial.print(I_reading);
  Input2 = (I_reading - izerooffset) * 10 / ical; //current feedback in 0.1A
  //Serial.print(" I_ip:");
  //Serial.print(Input2);
  PID2.Compute();
  //Serial.print(" I_op:");
  //Serial.print(Output2);

  //  if (digitalRead(DriveSwPin)) {
  //    Serial.print(" Gear:D ");
  //  } else if (digitalRead(RevSwPin)) {
  //    Serial.print(" Gear:R ");
  //  } else {
  //    Serial.print(" Gear:N ");
  //  }

  //take minimum loop output
  if (abs(Output1) <= abs(Output2)) {
    //position mode
    currentLimiting = false;
    pwm = Output1;
    //Serial.print(" Mode:Pos");
    CC_iterations = 0;
  }
  else {
    //i lim mode
    currentLimiting = true;
    pwm = Output2;
    if (Output1 < 0) {
      pwm = Output2 * -1; //current loop is unidirectional so flip it for constant negative current output (CC reverse)
    }
    //Serial.println(" Mode:CC");
    CC_iterations++;
    //Serial.println(CC_iterations);
  }

  if (CC_iterations > (1000 * lockout_time) / interval) {
    lockout = true; //latch into lockout mode
  }
  if (lockout) {
    pwm = 0;
    digitalWrite(EstopPin, HIGH);
    //Serial.println("LOCKOUT");
  }

  if (pwm > 0) {
    analogWrite(RPWMpin, pwm);
    analogWrite(LPWMpin, 0);
  } else {
    analogWrite(RPWMpin, 0);
    analogWrite(LPWMpin, abs(pwm));
  }
}

void loop() {
  unsigned long currentMillisA = millis(); // store the current time

  //read all analogue in to smoothing function
  SteeringWheelVal.add(analogRead(SteeringWheelPin));
  SteeringFeedbackVal.add(analogRead(SteeringFeedbackPin) + SteerCentreOffset);
  AccelPedalVal.add(analogRead(LeftPedalPin));

  if (currentMillisA - previousMillisA >= interval) { // check if inteval passed
    previousMillisA = currentMillisA;   // save the last time you blinked the LED

    if (digitalRead(LocRemSwPin)) { //remote mode
      while (Serial.available() > 0) {
        int posraw = Serial.parseInt();
        if (Serial.read() == '\n') {
          pos = constrain(posraw, -100, 100);
        }
      }
    } else { //local mode
      pos = constrain(map(SteeringWheelVal.get(), 0, 1024, -100, 100), -100, 100);
    }
    wiperServo(pos);              // tell servo to go to position in variable 'pos'
    //Serial.print(" Pedal_raw:");
    //Serial.print(AccelPedalVal.getLast());
    //Serial.print(" Pedal_avg:");
    //Serial.println(AccelPedalVal.get() - PedalCentre);

    if (analogRead(BrakeHallPin) > 200) {
      //Serial.println("Brake On");
      manualBraking = true;
      drvcmd = 0;
      brkcmd = 0;
      Send(0, drvcmd, brkcmd, currentDriveMode); //stop if manual braking
    } else { //brake is not on
      manualBraking = false;
      //TODO: and check if in local mode
      if (lockout) {
        drvcmd = 0;
        brkcmd = 0;
        Send(0, drvcmd, brkcmd, currentDriveMode); //stop if steering fault
      }
      else { //no steering faults
        if (AccelPedalVal.get() - PedalCentre > pedaldeadband) {
          drvcmd = map(AccelPedalVal.get() - PedalCentre, pedaldeadband, (1023 - PedalCentre), 0, 1000);
          brkcmd = 0;
          //hoverbaord firmware input range is -1000 to 1000
          if (digitalRead(DriveSwPin)) {
            Send(0, drvcmd, brkcmd, currentDriveMode);
          } else if (digitalRead(RevSwPin)) {
            drvcmd = -drvcmd * revspd;
            Send(0, drvcmd, brkcmd, currentDriveMode);
          }
          else {
            drvcmd = 0;
            brkcmd = 0;
            Send(0, drvcmd, brkcmd, currentDriveMode);
          }
        } else if (AccelPedalVal.get() - PedalCentre < (0 - pedaldeadband)) { //brake
          brkcmd = map(PedalCentre - AccelPedalVal.get(), pedaldeadband, (PedalCentre), 0, 1000); //500 = full brake
          drvcmd = 0;
          Send(0, drvcmd, brkcmd, currentDriveMode);
          //Serial.print("sentbrake: ");
          //Serial.println(brkcmd);
        }
        else {
          drvcmd = 0;
          brkcmd = 0;
          Send(0, drvcmd, brkcmd, currentDriveMode); //stop if no pedal input
        }
      }
    }
    steeringtelem();
  }
  //delay(interval);

}

void steeringtelem() {

  //structure: $STEER,INPUT,GEAR,MANUALBRAKE,PEDALAVG,STEERSP,STEERIP,STEEROP,CURRENTIP,CURRENTOP,CURRENTLIMITING,LOCKOUT,SENTSPEED,SENTBRAKE*AA
  // $STEER,0,D,0,-9,-63,0,0,0.67,0,0,0,0,0*2E
  
  char buf[64];
  sprintf(buf, "$STEER");

  //INPUT: 1/0 (1 is remote, 0 is local)
  sprintf(buf, "%s,%d", buf, digitalRead(LocRemSwPin));

  //GEAR: D/N/R
  if (digitalRead(DriveSwPin)) {
    sprintf(buf, "%s,D", buf);
  } else if (digitalRead(RevSwPin)) {
    sprintf(buf, "%s,R", buf);
  } else {
    sprintf(buf, "%s,N", buf);
  }

  //manualbrake 0/1 (1 is braking)
  sprintf(buf, "%s,%d", buf, manualBraking);

  //pedalavg (-550 (e-brake) - 0 - ~+500 (full accel))
  sprintf(buf, "%s,%d", buf, AccelPedalVal.get() - PedalCentre);

  //steersp -100 (right) to 100 (left)
  sprintf(buf, "%s,%d", buf, pos);

  //steerip (feedback)
  sprintf(buf, "%s,%d", buf, Input1);

  //steerop (loop output)
  sprintf(buf, "%s,%d", buf, Output1);

  //currentip (0.1 amps) (note this is decimal to 2dp)
  char str_currentip[6];
  dtostrf(Input2, 4, 2, str_currentip);
  sprintf(buf, "%s,%d", buf, 0); //is this the null bug?  sprintf(buf, "%s,%s", buf, str_currentip)

  //currentop (loop output)
  sprintf(buf, "%s,%d", buf, Output2);

  //currentlimiting 0/1 1 is limiting
  sprintf(buf, "%s,%d", buf, currentLimiting);

  //lockout 0/1 1 is locked out
  sprintf(buf, "%s,%d", buf, lockout);

  //sentspeed (0-1000)
  sprintf(buf, "%s,%d", buf, drvcmd);

  //sentbrake (0-1000)
  sprintf(buf, "%s,%d", buf, brkcmd);

  //checksum
  sprintf(buf, "%s*%02X", buf, nmea0183_checksum(buf));

  Serial.println(buf);
}

/**
   Divides a given PWM pin frequency by a divisor.
   The resulting frequency is equal to the base frequency divided by
   the given divisor:
     - Base frequencies:
        o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
        o The base frequency for pins 5 and 6 is 62500 Hz.
     - Divisors:
        o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
          256, and 1024.
        o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
          128, 256, and 1024.
   PWM frequencies are tied together in pairs of pins. If one in a
   pair is changed, the other is also changed to match:
     - Pins 5 and 6 are paired on timer0
     - Pins 9 and 10 are paired on timer1
     - Pins 3 and 11 are paired on timer2
   Note that this function will have side effects on anything else
   that uses timers:
     - Changes on pins 3, 5, 6, or 11 may cause the delay() and
       millis() functions to stop working. Other timing-related
       functions may also be affected.
     - Changes on pins 9 or 10 will cause the Servo library to function
       incorrectly.
   Thanks to macegr of the Arduino forums for his documentation of the
   PWM frequency divisors. His post can be viewed at:
     https://forum.arduino.cc/index.php?topic=16612#msg121031
*/
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}


#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;
  // ignore the first $ sign,  no checksum in sentence
  for (i = 1; i < strlen(nmea_data); i ++) { // removed the - 3 because no cksum is present
    crc ^= nmea_data[i];
  }
  return crc;
}
