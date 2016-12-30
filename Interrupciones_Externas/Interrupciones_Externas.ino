#include <avr/io.h>
#include <avr/interrupt.h>
#include "pins_arduino.h"
volatile int encoder;
void Sleep_Init()
{
   SMCR |=(1<<SM0)|(1<<SM1)|(1<<SM2); //Extended Standby
   SMCR |=(1<<SE); //Enable sleep mode
   __asm__ __volatile__ ("sleep" "\n\t" :: );
}

void setup() {
  // put your setup code here, to run once:
  pinMode(8,OUTPUT);
  pinMode(9,INPUT);
 PCMSK0 |= (1<<PCINT5); //Use PCINT6 = PB6 pin 30.
   PCICR |= (1<<PCIE0);   //Enable Pin change interrupt
   sei();
}

void loop() {
  // put your main code here, to run repeatedly:
Sleep_Init();
}
ISR(PCINT0_vect)
{
   SMCR &=~(1<<SE);  //Disable sleep mode
   PORTB ^=_BV(PB4); //Toggle LED (for testing)
}
