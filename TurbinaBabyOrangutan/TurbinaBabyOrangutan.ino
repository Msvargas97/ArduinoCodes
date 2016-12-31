/*
 * Codigo optimizado para el uso de Turbinas EDF27 o control de 1 solo motor BLCD
 * Mediante el uso del ESC (Electronic Speed Controller)
 * Este codigo funciona solamente con la Baby Orangutan
 * 
 * By Michael Vargas
 * 30/12/2016
 * Los Pin 9 y 10 quedan
 * Deshabilitados SOLO para el uso del PWM 
 * se puede usar como I/O sin problema
 * 
 * Creditos:
 * https://www.pololu.com/docs/0J57/8.a
 */

#define F_CPU 20000000UL

#define ESC_PIN 7 //Pin digital donde esta conectado el ESC 
#define ESC_SPEED   2000

#define ESC_Stop()   ESCSetPosition(1500)      //Envia señal de 1.5 ms para detener el motor
#define ESC_Start()  ESCSetPosition(ESC_SPEED) //Envia señal de xx ms para girar el motor

//########## Variables Timer1 - ESC ############
uint16_t volatile ESCTime = 0;
uint16_t volatile ESC_HighTime = 3750; //No cambiar
boolean volatile ESCHigh = false;
//##############################################

void setup()
{
  ESCInit();
  delay(2000);
}
void loop()
{
  ESC_Start();
  delay(2000);
  ESC_Stop();
  delay(2000);
}

// This ISR runs after Timer 1 reaches OCR1A and resets.
// In this ISR, we set OCR1A in order to schedule when the next
// interrupt will happen.
// Generally we will set OCR1A to 255 so that we have an
// interrupt every 102.4 us, but the first two interrupt intervals
// after the rising edge will be smaller so we can achieve
// the desired pulse width.
ISR(TIMER1_COMPA_vect)
{
  // The time that passed since the last interrupt is OCR1A + 1
  // because the timer value will equal OCR1A before going to 0.
  ESCTime += OCR1A + 1;

  static uint16_t highTimeCopy = 3750;
  static uint8_t interruptCount = 0;

  if (ESCHigh)
  {
    if (++interruptCount == 2)
    {
      OCR1A = 255;
    }

    // The ESC pin is currently high.
    // Check to see if is time for a falling edge.
    // Note: We could == instead of >=.
    if (ESCTime >= highTimeCopy)
    {
      // The pin has been high enough, so do a falling edge.
      digitalWrite(ESC_PIN, LOW);
      ESCHigh = false;
      interruptCount = 0;
    }
  }
  else
  {
    // The ESC pin is currently low.

    if (ESCTime >= 50000)
    {
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = ESC_HighTime;
      digitalWrite(ESC_PIN, HIGH);
      ESCHigh = true;
      ESCTime = 0;
      interruptCount = 0;
      OCR1A = ((highTimeCopy % 256) + 256) / 2 - 1;
    }
  }
}

void ESCInit()
{
  digitalWrite(ESC_PIN, LOW);
  pinMode(ESC_PIN, OUTPUT);

  // Turn on CTC mode.  Timer 2 will count up to OCR2A, then
  // reset to 0 and cause an interrupt.
  //  TCCR1A = ;
  // Set a 1:8 prescaler.  This gives us 0.5us resolution.
  TCCR1B = (1 << CS11) | (1 << WGM12);

  // Put the timer in a good default state.
  TCNT1 = 0;
  OCR1A = 255;

  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt.
  sei();   // Enable interrupts.
  
  ESCSetPosition(1000);
  delay(150);
  ESCSetPosition(1000);
}

void ESCSetPosition(uint16_t highTimeMicroseconds)
{
  TIMSK1 &= ~(1 << OCIE1A); // disable timer compare interrupt
  ESC_HighTime = highTimeMicroseconds * 2.5;
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
}
