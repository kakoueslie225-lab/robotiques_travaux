
#include <MeAuriga.h>
#include <Wire.h>

#define ALL_LEDS 0
#define LEDNUM 12
#define LED_PIN 44

/* Orion : Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);

int speed = 255 * 0.70;
int halfPwm = speed / 2;
const int turnPwm = 100;

//Motor Left
const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;


const int detection_min = 30;
const int detection_max = 100;


int long_delay = 2000;
int short_delay = 1000;
int turn_delay = 900;

unsigned long currentTime = 0;

unsigned long previousTime = 0;

unsigned long previousReadTime = 0;
unsigned long previousPrintTime = 0;

const int print_Rate = 250;
const int reading_Rate = 100;

enum state { FORWARD,
             SLOWING_DOWN,
             REVERSING,
             STOP,
             TURNING } currentState = FORWARD;

float distance;


void setup() {
  led.setpin(LED_PIN);
  Serial.begin(115200);
  pinMode(m1_pwm, OUTPUT);
  pinMode(m1_in2, OUTPUT);
  pinMode(m1_in1, OUTPUT);

  pinMode(m2_pwm, OUTPUT);
  pinMode(m2_in2, OUTPUT);
  pinMode(m2_in1, OUTPUT);
}

void loop() {

  currentTime = millis();
  distance = readDistance(currentTime);
  printDistance();
  stateManager();
}

float readDistance(unsigned long currentTime) {

  if (currentTime - previousReadTime >= reading_Rate) {
    previousReadTime = currentTime;
    float d = ultraSensor.distanceCm();
    if (d > 0) distance = d ;
  }
  return distance;
}

void printDistance() {
  if (currentTime - previousPrintTime >= print_Rate) {
    previousPrintTime = currentTime;
    Serial.print("Distance : ");
    Serial.print(ultraSensor.distanceCm());
    Serial.println(" cm");
  }
}
void normalDrive() {
  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, speed);  //Set speed via PWM

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, speed);  //Set speed via PWM

  for (uint8_t t = 0; t < LEDNUM / 2; t++) {
    led.setColorAt(t, 0, 255, 0);
  }
  led.show();
  led.setColor(0, 0, 0);
}


void stop() {
  analogWrite(m1_pwm, 0);
  analogWrite(m2_pwm, 0);

  for (uint8_t i = 0; i < LEDNUM; i++) {
    led.setColorAt(i, 255, 0, 0);
  }
  led.show();
  led.setColor(0, 0, 0);
}

void reduceSpeed() {

  digitalWrite(m1_in2, LOW);
  digitalWrite(m1_in1, HIGH);
  analogWrite(m1_pwm, halfPwm);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, halfPwm);

  for (uint8_t i = 0; i < LEDNUM / 2; i++) {
    led.setColorAt(i, 255, 255, 0);
  }
  led.show();
  led.setColor(0, 0, 0);
}
void reverse() {

  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, halfPwm);

  digitalWrite(m2_in2, HIGH);
  digitalWrite(m2_in1, LOW);
  analogWrite(m2_pwm, halfPwm);
}

void turn() {

  digitalWrite(m1_in2, HIGH);
  digitalWrite(m1_in1, LOW);
  analogWrite(m1_pwm, turnPwm);

  digitalWrite(m2_in2, LOW);
  digitalWrite(m2_in1, HIGH);
  analogWrite(m2_pwm, turnPwm);
}

void stateManager() {
  switch (currentState) {

    case FORWARD:
      if (distance < detection_min) {
        currentState = STOP;
        previousTime = currentTime;
      } else if (distance < detection_max) {
        currentState = SLOWING_DOWN;
      } else {
        normalDrive();
      }
      break;

    case SLOWING_DOWN:
      if (distance < detection_min) {
        currentState = STOP;
        previousTime = currentTime;
      } else if (distance >= detection_max) {
        currentState = FORWARD;
      } else {
        reduceSpeed();
      }
      break;

    case STOP:
      stop();
      if (currentTime - previousTime >= long_delay) {
        currentState = REVERSING;
        previousTime = currentTime;
      }
      break;

    case REVERSING:
      reverse();
      if (currentTime - previousTime >= short_delay) {
        currentState = TURNING;
        previousTime = currentTime;
      }
      break;

    case TURNING:
      turn();
      if (currentTime - previousTime >= turn_delay) {
        currentState = FORWARD;
      }
      break;

    default:
      currentState = STOP;
      previousTime = currentTime;
  }
}