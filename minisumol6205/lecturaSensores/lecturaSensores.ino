uint8_t sensors = 0b101111;
void setup() {
  // put your setup code here, to run once:
SensorsInit();
setupPWM();
 delay(3000);
digitalWrite(12,HIGH);
digitalWrite(6,LOW);
//analogWrite(9,i);
digitalWrite(13,HIGH);
digitalWrite(5,LOW);
OCR1A = OCR1B = 512;
  TCCR3A = TCCR3C = 0;
  TCCR3B = 0b00000010;
  TIMSK3 |= (1 << TOIE3);
  interrupts();
  TCNT3 = 49536;

}

void loop() {
  sensors = readSensors();
  _delay_ms(2);
}
ISR(TIMER3_OVF_vect) { 
  
}
uint8_t readSensors(){
    if (!bit_is_set(PINB, 7)) PORTD |= _BV(5);
  else PORTD &= ~_BV(5);
  if (!bit_is_set(PINE, 6))PORTD |= _BV(0);
  else PORTD &= ~_BV(0);
  if (!bit_is_set(PINF, 0))PORTD |= _BV(1);
  else PORTD &= ~_BV(1);
  if (!bit_is_set(PINF, 1))PORTD |= _BV(2);
  else PORTD &= ~_BV(2);
  if (!bit_is_set(PINF,4))PORTD |= _BV(3);
  else PORTD &= ~_BV(3);
  return (PIND & 0b101111) ;
}
void setupPWM() {
  PORTB &= (~_BV(5) & ~_BV(6));
  DDRB |= (_BV(5) | _BV(6));
  DDRC |= (_BV(6) | _BV(7));
  DDRD |= (_BV(6) | _BV(7));
  
  //Phase Correct PWM 8 bits WGM10 = 1
  TCCR1A = (1<<COM1B1) | (1<<COM1A1) | (1<<WGM11) | (1<<WGM10) ; //Activa FastPWM 10 bits
  TCCR1B =  (1<<CS11) ;//prescaler 1 Fpwm = 15.68 KHz
  OCR1A = OCR1B = 0;
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

