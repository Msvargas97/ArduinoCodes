#include <avr/interrupt.h>
#include <Servo.h>
Servo turbina;

#define PIN_ESC     13
#define PIN_MOTR    10
#define PWM_R       OCR1B
#define PIN_MOTL    11
#define PWM_L       OCR2A
#define BTN_LED     12

const int maxPulse = 2000;
const int minPulse = 1000;

void setup() {
  // put your setup code here, to run once:
  setupPWM();
  ADCSRA &= ~(1 << ADEN);

  pinMode(BTN_LED, INPUT);
  DDRB |= (1 << 6);
  turbina.attach(PIN_ESC, minPulse, maxPulse);
  InitESC();
  while (digitalRead(BTN_LED));
  delay(200);
  turbina.write(1200);
  pinMode(BTN_LED, OUTPUT);
  digitalWrite(12, LOW);
}

void loop() {
  for (int i = 0; i < 256; i++) {
    setSpeeds(i, i);
    delay(100);
  }
  // setSpeeds(70,0);
  //delay(10000);
  //setSpeeds(0,70);
  //delay(10000);
}

void setupPWM() {
  DDRB |= _BV(2) | _BV(3);
  //Fast PWM de 8 bits WGM12 = 1 WGM10 = 1
    TCCR1A = _BV(COM1B1) | _BV(COM1B0)  | _BV(WGM10); //Activa FastPWM 8 bits
    TCCR1B |= _BV(WGM12) | _BV(CS11); //Prescaler de 8 Fpwm = 4KHz
  //  //Fast PWM
  TCCR2A = _BV(COM2A0) | _BV(COM2A1) | //Configura modo invertido
           _BV(WGM21) | _BV(WGM20); //Activa Fast PWM
  TCCR2B = _BV(CS21); //prescaler 8 Fpwm = 4KHz
  PORTB |= (1 << 2) | (1 << 3);
}
static inline void setSpeeds(unsigned char left, unsigned char right) {
  if (left == 0) PORTB |= (1 << 3);
  else if (left == 0xFF) PORTB &= ~(1 << 3);
  else PWM_L = left;
  if (right == 0) PORTB |= (1 << 2);
  else if (right == 0xFF) PORTB &= ~(1 << 2);
  else {
    //    DDRB |= _BV(2);
    //    TCCR1A = _BV(COM1B1) | _BV(COM1B0) ;
    //    PWM_R = right;
    analogWrite(10, right);
  }
}
void InitESC() {
  turbina.write(0);
  delay(150);
  turbina.write(0);
}
