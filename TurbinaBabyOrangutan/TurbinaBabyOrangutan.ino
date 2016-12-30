// This line specifies what pin we will use for sending the
// signal to the servo.  You can change this.
#define F_CPU 20000000UL
#define SERVO_PIN 11

// This is the time since the last rising edge in units of 0.5us.
uint16_t volatile servoTime = 0;

// This is the pulse width we want in units of 0.5us.
uint16_t volatile servoHighTime = 3750;

// This is true if the servo pin is currently high.
boolean volatile servoHigh = false;

void setup()
{
  servoInit();
  servoSetPosition(1000);
  delay(150);
  servoSetPosition(1000);
}

void loop()
{
  servoSetPosition(1500);
  delay(2000);
  servoSetPosition(2000);  // Send 2000us pulses.
  delay(2000);
}

// This ISR runs after Timer 2 reaches OCR2A and resets.
// In this ISR, we set OCR2A in order to schedule when the next
// interrupt will happen.
// Generally we will set OCR2A to 255 so that we have an
// interrupt every 128 us, but the first two interrupt intervals
// after the rising edge will be smaller so we can achieve
// the desired pulse width.
ISR(TIMER1_COMPA_vect)
{
  // The time that passed since the last interrupt is OCR2A + 1
  // because the timer value will equal OCR2A before going to 0.
  servoTime += OCR1A + 1;

  static uint16_t highTimeCopy = 3750;
  static uint8_t interruptCount = 0;

  if (servoHigh)
  {
    if (++interruptCount == 2)
    {
      OCR1A = 255;
    }

    // The servo pin is currently high.
    // Check to see if is time for a falling edge.
    // Note: We could == instead of >=.
    if (servoTime >= highTimeCopy)
    {
      // The pin has been high enough, so do a falling edge.
      digitalWrite(SERVO_PIN, LOW);
      servoHigh = false;
      interruptCount = 0;
    }
  }
  else
  {
    // The servo pin is currently low.

    if (servoTime >= 50000)
    {
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = servoHighTime;
      digitalWrite(SERVO_PIN, HIGH);
      servoHigh = true;
      servoTime = 0;
      interruptCount = 0;
      OCR1A = ((highTimeCopy % 256) + 256) / 2 - 1;
    }
  }
}

void servoInit()
{
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);

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
}

void servoSetPosition(uint16_t highTimeMicroseconds)
{
  TIMSK1 &= ~(1 << OCIE1A); // disable timer compare interrupt
  servoHighTime = highTimeMicroseconds * 2.5;
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
}
