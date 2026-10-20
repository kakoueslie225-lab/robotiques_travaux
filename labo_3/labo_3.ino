#include <MeAuriga.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

#define ALL_LEDS 0
#define LEDNUM 12
#define LED_PIN 44

/* Orion : Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);
MeGyro gyro(0, 0x69);

const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;
bool ledState = false;

unsigned long currentTime = 0;

const float diametre_roue = 6.5;
const float dist_roue = 15.0;
const float ticks_par_tour = 9 * 39.267;
const int delay_start = 3000;
int circ = 48.7;
const int vitesse = 50;
unsigned long previousTime = 0;
const int ledDelay = 50;
bool flag = false;
float distance;
const int detection_dist = 30;
unsigned long previousReadTime = 0;
const int reading_Rate = 100;
bool enMarche = false;
bool termine = false;
bool lecture = false;
bool pivotStart = false;
unsigned long debutEtat = 0;

const int pwmDroit = 100;
long start1 = 0, start2 = 0;
long cibleDeg = 0;

enum state {
  DEPART,
  SEG_A,
  PIVOT,
  SEG_B,
  REVERSE,
  SEG_C,
  DEMI_TOUR,
  FIN
} currentState = DEPART;

void setup() {
  // put your setup code here, to run once:
  led.setpin(LED_PIN);
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  gyro.begin();

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(9);
  Encoder_2.setPulse(9);
  Encoder_1.setRatio(39.267);
  Encoder_2.setRatio(39.267);
  Encoder_1.setPosPid(1.8, 0, 1.2);
  Encoder_2.setPosPid(1.8, 0, 1.2);
  Encoder_1.setSpeedPid(0.18, 0, 0);
  Encoder_2.setSpeedPid(0.18, 0, 0);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = millis();
  float angle = 90;
  gyro.update();
  readDistance(currentTime);
  Encoder_1.loop();
  Encoder_2.loop();
  stateManager();
  //pour mon gyro j ai juste besoin de getangleZ
}

void isr_process_encoder1(void) {
  if (digitalRead(Encoder_1.getPortB()) == 0) {
    Encoder_1.pulsePosMinus();
  } else {
    Encoder_1.pulsePosPlus();
    ;
  }
}

void isr_process_encoder2(void) {
  if (digitalRead(Encoder_2.getPortB()) == 0) {
    Encoder_2.pulsePosMinus();
  } else {
    Encoder_2.pulsePosPlus();
    ;
  }
}

float distanceToTicks(float distance) {
  return (long)(distance / (PI * diametre_roue) * 360);
}
bool mouvementTermine() {
  return Encoder_1.isTarPosReached() && Encoder_2.isTarPosReached();
}

void changerEtat(state etatSuivant) {
  currentState = etatSuivant;
  debutEtat = currentTime;
  flag = false;
  pivotStart = false;
  enMarche = false;
  termine = false;
  lecture = false;
}

void stateManager() {
  gererLed();
  switch (currentState) {
    case DEPART:
      if (currentTime - debutEtat >= delay_start) changerEtat(SEG_A);
      break;
    case SEG_A:
      segment_1(80);
      lightOn(80);
      if (termine) changerEtat(PIVOT);
      break;
    case PIVOT:
      pivot(90);
      if (pivotStart && mouvementTermine()) changerEtat(SEG_B);
      break;
    case SEG_B:
      segment_2();
      if (termine) {
        changerEtat(REVERSE);
      }
      break;
    case REVERSE:
      pivot_Reverse();
      if (pivotStart && mouvementTermine()) changerEtat(SEG_C);
      break;
    case SEG_C:
      segment_3();
      if (termine) changerEtat(DEMI_TOUR);
      break;
    case DEMI_TOUR:
      semiPivot();
      if (pivotStart && mouvementTermine()) changerEtat(FIN);
      break;
    case FIN:
      ligneFinale(currentTime);
      break;
  }
}

void allerDroit(short speed = 100, short firstRun = 0) {
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  const double kp = 3.0;
  const double kd = 1.0;
  if (firstRun) {
    gyro.resetData();
    zAngleGoal = gyro.getAngleZ();
    firstRun = 0;
    Encoder_1.setTarPWM(speed);
    Encoder_2.setTarPWM(-speed);
    return;
  }
  error = gyro.getAngleZ() - zAngleGoal;
  output = kp * error + kd * (error - previousError);
  previousError = error;
  Encoder_1.setMotorPwm(speed - output);
  Encoder_2.setMotorPwm(-speed - output);
}

void debutSegment(float distanceCm) {
  cibleDeg = distanceToTicks(distanceCm);
  start1 = Encoder_1.getCurPos();
  start2 = Encoder_2.getCurPos();
  Encoder_1.setMotionMode(PWM_MODE);
  Encoder_2.setMotionMode(PWM_MODE);
}

float degresParcourus() {
  return (abs(Encoder_1.getCurPos() - start1) + abs(Encoder_2.getCurPos() - start2)) / 2.0;
}

void arreter() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  Encoder_1.setTarPWM(0);
  Encoder_2.setTarPWM(0);
}

void segment_1(float distance) {
  if (!flag) {
    debutSegment(distance);
    allerDroit(pwmDroit, 1);
    flag = true;
    return;
  }
  if (termine) return;
  if (degresParcourus() >= cibleDeg) {
    arreter();
    termine = true;
    return;
  }
  allerDroit(pwmDroit);
}


//modifier avec les nouvelles formules du cours 8
void pivot(float angle) {
  if (pivotStart) return;
  float angleMoteur = (angle * dist_roue) / diametre_roue;
  Encoder_1.setMotionMode(PID_MODE);
  Encoder_2.setMotionMode(PID_MODE);
  Encoder_1.move(angleMoteur, vitesse);
  Encoder_2.move(angleMoteur, vitesse);
  pivotStart = true;
}

float readDistance(unsigned long currentTime) {

  if (currentTime - previousReadTime >= reading_Rate) {
    previousReadTime = currentTime;
    float d = ultraSensor.distanceCm();
    if (d > 0) {
      distance = d;
      lecture = true;
    }
  }
  return distance;
}

void segment_2() {
  if (termine || !lecture) return;
  if (!enMarche) {

    float aParcourir = distance - detection_dist;
    if (aParcourir <= 0) {
      termine = true;
      return;
    }
    debutSegment(aParcourir);
    allerDroit(pwmDroit, 1);
    enMarche = true;
    return;
  }
  if (distance <= detection_dist || degresParcourus() >= cibleDeg) {
    arreter();
    termine = true;
    return;
  }
  allerDroit(pwmDroit);
}

void pivot_Reverse() {
}

void segment_3() {
}

void semiPivot() {
}

void ligneFinale(unsigned long currentTime) {
  if (currentTime - previousTime >= ledDelay) {
    previousTime = currentTime;
    ledState = !ledState;

    for (int i = 7; i < LEDNUM; i++) {
      if (ledState) {
        led.setColorAt(i, 0, 255, 0);
      } else {
        led.setColorAt(i, 0, 0, 0);
      }
    }
  }
  led.show();
}
void lightOn(float distance) {
  long cible = distanceToTicks(distance);

  // nombre de DEL à allumer (1 à 7) selon l'avancement
  long pos = abs(Encoder_1.getCurPos());
  int ledToLight = map(constrain(pos, 0, cible), 0, cible, 0, 7);

  for (uint8_t i = 0; i < 7; i++) {
    if (i <= ledToLight) {
      led.setColorAt(i, 255, 0, 0);
    } else {
      led.setColorAt(i, 0, 0, 0);
    }
  }

  led.show();
}
state dernierEtat = (state)-1;
void gererLed() {
  if (currentState == dernierEtat) return;
  dernierEtat = currentState;

  switch (currentState) {
    case SEG_A:
      led.setColorAt(11, 0, 255, 0);
      break;
    case PIVOT:
      for (uint8_t i = 0; i < 7; i++) {
        led.setColorAt(i, 0, 0, 0);
      }
      break;
    case SEG_B:
      led.setColorAt(10, 0, 255, 0);
      break;
  }
  led.show();
}