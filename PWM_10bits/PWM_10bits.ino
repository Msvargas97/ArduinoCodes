#include <avr/interrupt.h>

#define PWM9 OCR1A
#define PWM10 OCR1B
#define PWM5 OCR3A
#define PWM13L OCR4A
int speed;

void setup() {
  cli(); //Deshabilita las interrupciones
  DDRB |= ((1 << 5) | (1 << 6)); //Establece los pines como slaidas
  PORTB &= (~(1 << 5) & ~(1 << 6)); //Pone en bajos los pines de salida 9 y 10
  DDRC |= ((1 << 6) | (1 << 7)); //Establece los pines como slaidas
  PORTC &= (~(1 << 6) & ~(1 << 7)); //Pone en bajos los pines de salida 13 y 5
  //Configuracion de PWM - Fast PWM de 10 bits (1023) en los pines 9, 10 y 5
  TCCR3A = TCCR3B  = TCCR1A = TCCR1B = 0;
  TCCR1A = TCCR3A  = ((1 << COM1A1) | (1 << COM1B1) | (1 << WGM11) | (1 << WGM10)); //Selecciona los pines pwm de salida
  TCCR1B = TCCR3B  = ((1 << WGM12) | (1 << CS10)); //Seleciona prescaler de 1 -> Freq= 15Khz

  //Configuracion timer4 PWM 10bits
  TCCR4A = TCCR4B = TCCR4C = TCCR4D = TCCR4E = 0;
  TCCR4A = ((1 << COM4A1) | (1 << PWM4A)); //Activa la salida del pin 13
  PLLFRQ &= (~(1 << PLLTM1) & ~(1 << PLLTM0)); // Deshabilita el postcaler del PLL
  //PLLFRQ = (PLLFRQ & 0xCF) | 0x30; Postcaler de 48 Mhz
  TCCR4B = (1 << CS40); //Prescaler de 1
  TC4H = 0x03; //Asigna la resolucion
  OCR4C = 0xFF;
  PWM9 = PWM10 = PWM5 = PWM13L = 0;
  sei(); //Habilita las interrupciones
  digitalWrite(30, LOW);
}

void loop() {
  PWM9 = speed;
  TC4H = speed >> 8;
  PWM13L = speed & 0xFF;
  speed += 10;
  if (speed > 1023) speed = 0;
  delay(100);
}
