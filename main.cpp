#include <Arduino.h>
// #include <TimerOne.h>
#include <Servo.h>


int encoder0PinA = 2;
int encoder0PinB = 4;
int motorPWM = 6;
int motorDirA = 8;
int motorDirB = 7;
int ballServoPin = 11;
int sweepServoPin = 5;
int topMotorPWM = 3;
int topMotorDirA = 12;
int topMotorDirB = 13;

//int callPin = 9;  // For Timer1
int callPin;  //placeholder
int ticks_per_rev = 48;
double num_revs = 0;
double rev_thresh = 0.125;

int encoder0Pos = 0;   //ticks
double encoder0Vel = 0;   //rpm

//double pwm_freq = 25000;

unsigned long millisReset = 0;

int curr_time = 0;

int prev_vel_time = 0;
double prev_vel = 0;
double vel_des = 1500; //rpm

int prev_release_time = 0;
int time_between_releases = 2000;
int release_start_time = 0;

long int ti;
volatile bool intFlag=false;

Servo ballReleaser;
const int servoClosed = 145;
const int servoOpen = 180;
const int BALL_RELEASE_INTERVAL = 60;

Servo sweeper;
const int servoLeft = 60;
const int servoRight = 120;
const int LEFT = 0;
const int RIGHT = 1;
int lastSweep = LEFT;
int prevSweepTime = 0;
int sweepPos = 0;
int sweepDelay = 15;  //ms

int printCount = 0;

typedef enum {NOT_START, WAITING, SWEEPING, RELEASING} ServoStates;
typedef enum {BACKSPIN, TOPSPIN} WheelStates;

ServoStates servoState;
const ServoStates STARTING_STATE = NOT_START;

WheelStates wheelState = BACKSPIN;

void doEncoderA();
void doEncoderB();
void callback();
void feedbackControl();
void printState();

void setup() {
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirA, OUTPUT);
  pinMode(motorDirB, OUTPUT);
  analogWrite(motorPWM, 0); //make sure motor is off

  pinMode(topMotorPWM, OUTPUT);
  pinMode(topMotorDirA, OUTPUT);
  pinMode(topMotorDirB, OUTPUT);
  analogWrite(topMotorPWM, 0);

  // analogWriteFrequency(motorPWM, pwm_freq);  // for Teensy
  digitalWrite(motorDirA, LOW);
  digitalWrite(motorDirB, LOW);
  digitalWrite(topMotorDirA, LOW);
  digitalWrite(topMotorDirB, LOW);

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE);

  Serial.begin(9600);

  pinMode(callPin, OUTPUT);
  //Timer1.initialize(10000);         // initialize timer1, and set a 1/2 second period
  //Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  ballReleaser.attach(ballServoPin);
  ballReleaser.write(servoClosed);

  sweeper.attach(sweepServoPin);
  sweeper.write(servoLeft);
  sweepPos = servoLeft;

  // Store initial time
  ti=millis();
  prev_vel_time = ti;

  servoState = STARTING_STATE;

  digitalWrite(topMotorDirA, HIGH);
  digitalWrite(topMotorDirB, LOW);
  analogWrite(topMotorPWM, 100);
}

void loop() {

  if (curr_time >= 30000) {
    millisReset = millis();
    prev_vel_time = 0 - (curr_time - prev_vel_time);
    prev_release_time = 0 - (curr_time - prev_release_time);
    release_start_time = 0 - (curr_time - release_start_time);
    prevSweepTime = 0 - (curr_time - prevSweepTime);
    curr_time = 0;
    Serial.println("Swapped");
  } else {
    curr_time = millis() - millisReset - ti;
  }
  
  if (abs(encoder0Pos) >= rev_thresh * ticks_per_rev) {
    num_revs += rev_thresh * encoder0Pos / abs(encoder0Pos);
    encoder0Pos = 0;
  }
  if (abs(num_revs) >= rev_thresh) {
    // calculate velocity
    feedbackControl();
  }

  switch (servoState) {
    case NOT_START:
      if (abs(encoder0Pos) >= rev_thresh * ticks_per_rev) {
        servoState = WAITING;
        prev_release_time = curr_time;
        //printState();
      }
      break;
    case WAITING: 
      if (curr_time - prev_release_time >= time_between_releases) {
        servoState = SWEEPING;
        prevSweepTime = curr_time;
        //printState();
      }
      break;
    case SWEEPING:
      if (lastSweep == LEFT) {
        if (curr_time - prevSweepTime >= sweepDelay) {
          sweepPos ++;
          prevSweepTime = curr_time;
          sweeper.write(sweepPos);
        }
        if (sweepPos >= servoRight) {
          lastSweep = RIGHT;
          servoState = RELEASING;
          release_start_time = curr_time;
          ballReleaser.write(servoOpen);
        }
      } else {
        if (curr_time - prevSweepTime >= sweepDelay) {
          sweepPos --;
          prevSweepTime = curr_time;
          sweeper.write(sweepPos);
        }
        if (sweepPos <= servoLeft) {
          lastSweep = LEFT;
          servoState = RELEASING;
          release_start_time = curr_time;
          ballReleaser.write(servoOpen);
        }
      }
      //printState();
      break;
    case RELEASING:
      if (curr_time - release_start_time >= BALL_RELEASE_INTERVAL) {
        servoState = WAITING;
        ballReleaser.write(servoClosed);
        prev_release_time = curr_time;
        //printState();
      }
      break;
    default:
      Serial.println("Shouldn't be stateless bro");
  } 
}


void feedbackControl() {
  prev_vel = encoder0Vel;
  encoder0Vel = num_revs * 60.0 * 1000 / (curr_time - prev_vel_time);
  num_revs = 0;
  // Time alter...
  double rpmToRadPerS = 6.28/60;
  double accel = (encoder0Vel -  prev_vel)*rpmToRadPerS*1000.0
        /(curr_time - prev_vel_time);
  prev_vel_time = curr_time;
  double kp = 10;
  double kd = 0;
  double kp1 = 11.91;
  double duty = (vel_des*rpmToRadPerS * kp1 - kp * encoder0Vel*rpmToRadPerS - kd * accel)/255;
  // double duty = kp * (vel_des - encoder0Vel) + kd * (0 - accel); 
  if (printCount > 20) {
    Serial.println(encoder0Vel);
    printCount = 0;
  } else {
    printCount ++;
  }
  double dutyTemp = duty;
  duty = abs(duty);
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }
  double output = (int)(duty * 255);
  if (dutyTemp > 0) {
    digitalWrite(motorDirA, LOW);
    digitalWrite(motorDirB, HIGH);
  } else {
    digitalWrite(motorDirA, HIGH);
    digitalWrite(motorDirB, LOW);
  }
  analogWrite(motorPWM, output); 
}

void callback() { 
  intFlag=true;
  digitalWrite(callPin, digitalRead(callPin) ^ 1);
}

void doEncoderA() {
  //Serial.println("I'm here");
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {
    //Serial.println("A went high");
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {

    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}