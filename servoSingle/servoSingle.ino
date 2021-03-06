#define SERVO_PIN 11
uint32_t volatile us, ms;

// This is the time since the last rising edge in units of 0.5us.
uint16_t volatile servoTime = 0;

// This is the pulse width we want in units of 0.5us.
uint16_t volatile servoHighTime = 3000;

// This is true if the servo pin is currently high.
boolean volatile servoHigh = false;

void setup()
{
  servoInit();
  Serial.begin(9600);
}

void loop()
{
  Serial.print(micros());
  Serial.print("  ");
  Serial.println(get_us());

}
uint32_t get_ms() {
  unsigned long m;
  uint8_t oldSREG = SREG;
  cli();
  m = us / 2 / 1000;
  SREG = oldSREG;
  return m;
}
uint32_t get_us() {
  unsigned long m;
  uint8_t oldSREG = SREG;
  cli();
  m = us / 2;
  SREG = oldSREG;
  return m;
}
// This ISR runs after Timer 2 reaches OCR2A and resets.
// In this ISR, we set OCR2A in order to schedule when the next
// interrupt will happen.
// Generally we will set OCR2A to 255 so that we have an
// interrupt every 128 us, but the first two interrupt intervals
// after the rising edge will be smaller so we can achieve
// the desired pulse width.
ISR(TIMER2_COMPA_vect)
{

  // The time that passed since the last interrupt is OCR2A + 1
  // because the timer value will equal OCR2A before going to 0.
  servoTime += OCR2A + 1;
  us += OCR2A + 1;

  static uint16_t highTimeCopy = 3000;
  static uint8_t interruptCount = 0;

  if (servoHigh)
  {
    if (++interruptCount == 2)
    {
      OCR2A = 255;
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

    if (servoTime >= 40000)
    {
      // We've hit the end of the period (20 ms),
      // so do a rising edge.
      highTimeCopy = servoHighTime;
      digitalWrite(SERVO_PIN, HIGH);
      servoHigh = true;
      servoTime = 0;
      interruptCount = 0;
      OCR2A = ((highTimeCopy % 256) + 256) / 2 - 1;
    }
  }
}

void servoInit()
{
  digitalWrite(SERVO_PIN, LOW);
  pinMode(SERVO_PIN, OUTPUT);

  // Turn on CTC mode.  Timer 2 will count up to OCR2A, then
  // reset to 0 and cause an interrupt.
  TCCR2A = (1 << WGM21);
  // Set a 1:8 prescaler.  This gives us 0.5us resolution.
  TCCR2B = (1 << CS21);

  // Put the timer in a good default state.
  TCNT2 = 0;
  OCR2A = 255;

  TIMSK2 |= (1 << OCIE2A);  // Enable timer compare interrupt.
  sei();   // Enable interrupts.
}

void servoSetPosition(uint16_t highTimeMicroseconds)
{
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  servoHighTime = highTimeMicroseconds * 2;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt
}
