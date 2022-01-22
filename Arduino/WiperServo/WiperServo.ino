//input: -100% to 100% of steering range over serial
//todo: analog steering input
//todo: remote/local switch
//todo: detect failure to control
//todo: detect motor vin

#include <PID_v1.h>  //PID loop from http://playground.arduino.cc/Code/PIDLibrary

double Pk1 = 7;  //speed it gets there
double Ik1 = 0;
double Dk1 = 0;
int deadband = 5;
int centreoffset = 25;
int poslimit = 150; //150 is somewhat arbritrary number that represents 90 degrees each way

double Setpoint1, Input1, Output1, Output1a;    // PID variables

PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

volatile unsigned long pwm;
volatile boolean done;
unsigned long start;

int pot;
int sp;
int pos = 0;
int posraw = 0;

unsigned long currentMillis;

long previousMillis = 0;    // set up timers
long interval = 50;        // time constant for timers

void setup() {
  pinMode(2, INPUT);
  pinMode(A0, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  Serial.begin(115200);
  PID1.SetMode(AUTOMATIC);              // PID Setup - trousers SERVO
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(20);
}

void wiperServo(int sp) {
  Setpoint1 = constrain(map(sp, -100, 100, 0-poslimit, poslimit), 0-poslimit, poslimit); 
  pot = analogRead(A0) + centreoffset;
  Serial.print("pot:");
  Serial.print(pot);
  Serial.print(" sp:");
  Serial.print (Setpoint1);
  Serial.print(" ip:");
  Input1 = map(pot, 0, 1023, -255, 255);
  Serial.print (Input1);
  PID1.Compute();
  Serial.print(" op:");
  Serial.println(Output1);

  if (Output1 > deadband) {
    analogWrite(5, Output1);
    analogWrite(6, 0);
  }
  else if (Output1 < (0-deadband)) {
    Output1a = abs(Output1);
    analogWrite(5, 0);
    analogWrite(6, Output1a);
  }
  else {
    analogWrite(5, 0);
    analogWrite(6, 0);
  }
}

void loop() {
  while (Serial.available() > 0) {
    int posraw = Serial.parseInt();
    if (Serial.read() == '\n') {
      pos = constrain(posraw, -225, 225);
    }
  }
  wiperServo(pos);              // tell servo to go to position in variable 'pos'
  delay(interval);
}



