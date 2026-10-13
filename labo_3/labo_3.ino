#include <MeAuriga.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

#define ALL_LEDS 0
#define LEDNUM 12
#define LED_PIN 44

/* Orion : Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed led(0, LEDNUM);

const int m1_pwm = 11;
const int m1_in1 = 48;
const int m1_in2 = 49;

//Motor Right
const int m2_pwm = 10;
const int m2_in1 = 47;
const int m2_in2 = 46;
bool ledState = false;

unsigned long currentTime = 0;

const float diametre_roue = 6.4;
const float ticks_par_tour = 9 * 39.267;
const int delay_start = 3000;
int circ = 48.7;
const int vitesse = 100;
unsigned long previousTime = 0;
const int ledDelay = 50;

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
  Encoder_1.loop();
  Encoder_2.loop();
  segment_1(80);
  lightOn(80);
  //stateManager();
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
  return (long)(distance / (PI * diametre_roue) * ticks_par_tour);
}


void stateManager() {
  switch (currentState) {
    case DEPART:
      break;
    case SEG_A:
      segment_1(80);
      lightOn(80);
      break;
    case PIVOT:
      pivot(90);
      break;
    case SEG_B:
      segment_2();
      break;
    case REVERSE:
      pivot_Reverse();
      break;
    case SEG_C:
      segment_3();
      break;
    case DEMI_TOUR:
      semiPivot();
      break;
    case FIN:
      break;
  }
}
void segment_1(float distance) {
  float cible = distanceToTicks(distance);
  Encoder_1.moveTo(-cible, vitesse);
  Encoder_2.moveTo(cible, vitesse);
}
//modifier avec les nouvelles formules du cours 8
void pivot(float angle) {
  float distance_roue = (angle / 360) * circ;
  long cible = distanceToTicks(distance_roue);
  Encoder_1.moveTo(cible);
  Encoder_2.moveTo(cible);
}

void segment_2() {
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
      led.setColorAt(i, 0, 0, 0);  // éteinte
    }
  }
  led.setColorAt(11, 0, 255, 0);
  led.show();
}