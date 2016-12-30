#include <avr/interrupt.h> 
# include <avr/sleep.h>

void sleepATmega(){
   set_sleep_mode(SLEEP_MODE_PWR_SAVE);//Establecemos el modo de bajo consumo.
   cli();
  sleep_enable();
//  sleep_bod_disable();
   sei();
   sleep_cpu();
     digitalWrite(4,0);
   sleep_disable();
   sei();
}
void setup() {
  // put your setup code here, to run once:
  pinMode(4,OUTPUT);
  digitalWrite(4,1);
  pinMode(8,INPUT_PULLUP);
   PCMSK0 |= (1<<PCINT4); //Use PCINT6 = PB6 pin 30.
   PCICR |= (1<<PCIE0);   //Enable Pin change interrupt
   sei();
   sleepATmega();
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(4,1);
  delay(100);
  digitalWrite(4,0);
  delay(100);
}
ISR(PCINT0_vect)
{
  if(!bitRead(PINB,4)) sleep_disable();
   //PORTB ^=_BV(PB4); //Toggle LED (for testing)
}
