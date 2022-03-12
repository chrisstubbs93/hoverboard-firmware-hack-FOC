//input: -100% to 100% of steering range over serial
//todo: analog steering input
//todo: remote/local switch
//todo: detect failure to control
//todo: detect motor vin

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

#define SteeringWheelPin A0
#define SteeringFeedbackPin A1
#define LeftPedalPin A2
#define RightPedalPin A3
#define DividerPin A4
#define CurrentSensePin A5
#define BrakeHallPin A6

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


int deadband = 1;
int centreoffset = 25;
int poslimit = 150; //150 is somewhat arbritrary number that represents 90 degrees each way
float izerooffset = 343.0; //adc counts at 0 amps
float ical = 56.0;//adc counts per 1amp
double ilim = 18; //in 0.1A
unsigned long CC_iterations = 0; //number of iterations in constant current mode
int lockout_time = 10; // time in constant current before tripping to lockout mode
bool lockout = false;


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
int pos = 0;
int posraw = 0;
int I_reading = 0;

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 50;        // time constant for timers

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
  pinMode(AUX3pin, INPUT);

  pinMode(SteeringWheelPin, INPUT);
  pinMode(SteeringFeedbackPin, INPUT);
  pinMode(LeftPedalPin, INPUT);
  pinMode(RightPedalPin, INPUT);
  pinMode(DividerPin, INPUT);
  pinMode(CurrentSensePin, INPUT);
  pinMode(BrakeHallPin, INPUT);  
  
  setPwmFrequency(LPWMpin, 1); //62500hz?
  setPwmFrequency(RPWMpin, 1); //62500hz?
  Serial.begin(115200);
  PID1.SetMode(AUTOMATIC);              // PID posn control loop
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);
  
  PID2.SetMode(AUTOMATIC);              // PID constant current loop
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(20);
  
  digitalWrite(MotorEnPin,HIGH);
}

void wiperServo(int sp) {
  Setpoint1 = constrain(map(sp, -100, 100, 0-poslimit, poslimit), 0-poslimit, poslimit);
  pot = analogRead(SteeringFeedbackPin) + centreoffset;
  Serial.print(" pos_sp:");
  Serial.print (Setpoint1);
  Serial.print(" pos_ip:");
  Input1 = map(pot, 0, 1023, -255, 255);
  Serial.print (Input1);
  PID1.Compute();
  Serial.print(" pos_op:");
  Serial.print(Output1);
  I_reading = analogRead(CurrentSensePin);
  Serial.print("I_raw:");
  Serial.print(I_reading);
  Input2 = (I_reading-izerooffset)*10/ical; //current feedback in 0.1A
  Serial.print("I_ip:");
  Serial.print(Input2);
  PID2.Compute();
  Serial.print(" I_op:");
  Serial.print(Output2);
  
  if(digitalRead(DriveSwPin)) {
    Serial.print(" D ");
  } else if (digitalRead(RevSwPin)) {
    Serial.print(" R ");
  } else{
    Serial.print(" N ");
  }
  
  //take minimum loop output
  if (abs(Output1) <= abs(Output2)) {
    //position mode
    pwm = Output1;
    Serial.println(" pos mode");
    CC_iterations = 0;
  }
  else {
    //i lim mode
    pwm = Output2;
    if (Output1 < 0){pwm = Output2 * -1;} //current loop is unidirectional so flip it for constant negative current output (CC reverse)
    Serial.println(" CC mode");
    CC_iterations++;
    Serial.println(CC_iterations);
  }

  if (CC_iterations > (1000*lockout_time)/interval) {lockout = true;} //latch into lockout mode
  if (lockout) {
    pwm = 0;
    digitalWrite(EstopPin,HIGH);
    Serial.println("LOCKOUT");
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
  if(digitalRead(LocRemSwPin)){ //remote mode
  while (Serial.available() > 0) {
    int posraw = Serial.parseInt();
    if (Serial.read() == '\n') {
      pos = constrain(posraw, -100, 100);
    }
  }
  } else { //local mode
    pos = constrain(map(analogRead(SteeringWheelPin), 0, 1024, -100, 100), -100, 100);

  }
  wiperServo(pos);              // tell servo to go to position in variable 'pos'
  if(analogRead(BrakeHallPin)>200){
    Serial.println("Brake On");
  }
  delay(interval);
}


/**
 * Divides a given PWM pin frequency by a divisor.
 *
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 *
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 *
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 *
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   https://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
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

