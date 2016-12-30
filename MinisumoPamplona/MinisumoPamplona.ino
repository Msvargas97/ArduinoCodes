#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

uint8_t sensors = 0b101111;
void setup() {
  // put your setup code here, to run once:
  noInterrupts();
  // cbi(TIMSK0, TOIE0);
  setupTimer1();
  SensorsInit();
//  setupPWM();
  _delay_ms(12);
  pinMode(3,OUTPUT);
    pinMode(5,OUTPUT);
  pinMode(13,OUTPUT);

  interrupts();
  analogWrite(13,127);
  analogWrite(5,127);
  analogWrite(3,127);
//  TC4H = 512 >> 8;
//  OCR4A = 512 & 0xFF;
//  OCR3A = 512;
}

void loop() {

}
ISR(TIMER1_OVF_vect) {
  TCNT1 = 49536;
  readSensors(1, &sensors);
}
void readSensors(boolean leds, uint8_t *value) {
  *value  = 0;
  *value <<= 1;
  if (!bit_is_set(PINE, 6)) {
    if (leds) PORTD |= _BV(0);
    *value |= 1;
  }
  else {
    *value |= 0;
    if (leds) PORTD &= ~_BV(0);
  }
  *value <<= 1;
  if (!bit_is_set(PINF, 0)) {
    *value |= 1;
    if (leds) PORTD |= _BV(1);
  }
  else {
    *value |= 0;
    if (leds) PORTD &= ~_BV(1);
  }
  *value <<= 1;
  if (!bit_is_set(PINF, 1)) {
    *value |= 1;
    if (leds)  PORTD |= _BV(2);
  }
  else {
    *value |= 0;
    if (leds) PORTD &= ~_BV(2);
  }
  *value <<= 1;
  if (!bit_is_set(PINF, 4)) {
    *value |= 1;
    if (leds)  PORTD |= _BV(3);
  }
  else {
    *value |= 0;
    if (leds) PORTD &= ~_BV(3);
  }
  *value <<= 1;
  if (!bit_is_set(PINF,6)) {
    if (leds) PORTD |= _BV(5);
    *value |= 1;
  }
  else {
    *value |= 0;
    if (leds) PORTD &= ~_BV(5);
  }

}
void setupTimer1() {
  //Se ejecuta el timer3 cada 8ms para leer los sensores
  TCCR1A = TCCR1C = 0;
  TCCR1B = 0b00000010;
  TIMSK1 |= (1 << TOIE1);
  TCNT1 = 49536;
}
void setupPWM() {
  TCCR4A = TCCR4B = TCCR3A = TCCR3B = 0;
  PORTB &= (~_BV(5) & ~_BV(6));
  DDRB |= (_BV(5) | _BV(6));
  DDRC |= (_BV(6) | _BV(7));
  DDRD |= (_BV(6) | _BV(7));
  sbi(PORTB, 5);
  sbi(PORTB, 6);
  sbi(TCCR4B, CS42);    // set timer4 prescale factor to 64
  //  sbi(TCCR4B, CS41);
  //  sbi(TCCR4B, CS40);
  sbi(TCCR4D, WGM40);   // put timer 4 in phase- and frequency-correct PWM mode
  sbi(TCCR4A, PWM4A);   // enable PWM mode for comparator OCR4A
  TC4H = 0x03;
  OCR4C = 0xFF;
  sbi(TCCR3B, CS31);    // set timer 3 prescale factor to 8
  // sbi(TCCR3B, CS30);
  sbi(TCCR3B, WGM33);   // put timer 3 in 8-bit phase correct pwm mode
  sbi(TCCR3A,COM3A1);
  ICR3 = 1024;
}

void SensorsInit() {
  DDRE &= ~_BV(6);
  DDRF &= ~_BV(0) & ~_BV(1) & ~_BV(4);
  DDRB &= ~_BV(7);
  DDRD |= 0b101111;
  PORTE |= _BV(6);
  PORTF |= (_BV(0) | _BV(1) | _BV(4));
  PORTB |= _BV(7);
}
