//input: -100% to 100% of steering range over serial
//todo: analog steering input
//todo: remote/local switch
//todo: detect failure to control
//todo: detect motor vin

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

double Pk1 = 7;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;
int deadband = 1;
int centreoffset = 25;
int poslimit = 150; //150 is somewhat arbritrary number that represents 90 degrees each way
float izerooffset = 343.0; //adc counts at 0 amps
float ical = 56.0;//adc counts per 1amp
int ilim = 30; //in 0.1A
unsigned long CC_iterations = 0; //number of iterations in constant current mode
int lockout_time = 10; // time in constant current before tripping to lockout mode
bool lockout = false;

double Pk2 = 0;  //I lim
double Ik2 = 15;
double Dk2 = 0;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
double Setpoint2, Input2, Output2;    // PID variables

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID for posn control

PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID current limit

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
  pinMode(2, INPUT);
  pinMode(A0, INPUT); //pot
  pinMode(A1, INPUT);//L_IS
  pinMode(A2, INPUT);//R_IS
  pinMode(9, OUTPUT); //HBRIDGE IN1
  pinMode(10, OUTPUT); //HBRIDGE IN2
  setPwmFrequency(9, 1); //62500hz
  setPwmFrequency(10, 1); //62500hz
  Serial.begin(115200);
  PID1.SetMode(AUTOMATIC);              // PID posn control loop
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);
  
  PID2.SetMode(AUTOMATIC);              // PID constant current loop
  PID2.SetOutputLimits(-255, 255);
  PID2.SetSampleTime(20);
}

void wiperServo(int sp) {
  Setpoint1 = constrain(map(sp, -100, 100, 0-poslimit, poslimit), 0-poslimit, poslimit);
  Setpoint2 = ilim; //current limit in 0.1A
  pot = analogRead(A0) + centreoffset;
  Serial.print(" pos_sp:");
  Serial.print (Setpoint1);
  Serial.print(" pos_ip:");
  Input1 = map(pot, 0, 1023, -255, 255);
  Serial.print (Input1);
  PID1.Compute();
  Serial.print(" pos_op:");
  Serial.print(Output1);
  I_reading = analogRead(A1);
  Serial.print("I_raw:");
  Serial.print(I_reading);
  Input2 = (I_reading-izerooffset)*10/ical; //current feedback in 0.1A
  Serial.print("I_ip:");
  Serial.print(Input2);
  PID2.Compute();
  Serial.print(" I_op:");
  Serial.print(Output2);
  
  //take minimum loop output
  if (abs(Output1) < abs(Output2)) {
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
    Serial.println("LOCKOUT");
  }

  if (pwm > 0) {
    analogWrite(9, pwm);
    analogWrite(10, 0);
  } else {
    analogWrite(9, 0);
    analogWrite(10, abs(pwm));
  }
}

void loop() {
  while (Serial.available() > 0) {
    int posraw = Serial.parseInt();
    if (Serial.read() == '\n') {
      pos = constrain(posraw, -225, 225); //should this be -100-100????
    }
  }
  wiperServo(pos);              // tell servo to go to position in variable 'pos'
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

