//Timer4
#define PWM187k 1   // 187500 Hz
#define PWM94k  2   //  93750 Hz
#define PWM47k  3   //  46875 Hz
#define PWM23k  4   //  23437 Hz
#define PWM12k  5   //  11719 Hz
#define PWM6k   6   //   5859 Hz
#define PWM3k   7   //   2930 Hz
// Frequency modes for TIMER1
#define PWM62k   1   //62500 Hz
#define PWM8k    2   // 7812 Hz
#define PWM1k    3   //  976 Hz
#define PWM244   4   //  244 Hz
#define PWM61    5   //   61 Hz
// Direct PWM change variables
#define PWM6        OCR4D
#define PWM13       OCR4A
void pwm91011configure(int mode)
{
  // TCCR1A configuration
  //  00 : Channel A disabled D9
  //  00 : Channel B disabled D10
  //  00 : Channel C disabled D11
  //  01 : Fast PWM 8 bit
  TCCR1A = 1;

  // TCCR1B configuration
  // Clock mode and Fast PWM 8 bit
  TCCR1B = mode | 0x08;

  // TCCR1C configuration
  TCCR1C = 0;
}
void pwm5configure(int mode)
{
  // TCCR1A configuration
  //  00 : Channel A disabled D9
  //  00 : Channel B disabled D10
  //  00 : Channel C disabled D11
  //  01 : Fast PWM 8 bit
  TCCR3A = 1;

  // TCCR1B configuration
  // Clock mode and Fast PWM 8 bit
  TCCR3B = mode | 0x08;

  // TCCR1C configuration
  TCCR3C = 0;
}
void pwmSet6(int value)
{
  OCR4D = value; // Set PWM value
  DDRD |= 1 << 7; // Set Output Mode D7
  TCCR4C |= 0x09; // Activate channel D
}
void pwm613configure(int mode)
{
  // TCCR4A configuration
  TCCR4A = 0;

  // TCCR4B configuration
  TCCR4B = mode;

  // TCCR4C configuration
  TCCR4C = 0;

  // TCCR4D configuration
  TCCR4D = 0;

  // TCCR4D configuration
  TCCR4D = 0;

  // PLL Configuration
  // Use 96MHz / 2 = 48MHz
  PLLFRQ = (PLLFRQ & 0xCF) | 0x30;
  // PLLFRQ=(PLLFRQ&0xCF)|0x10; // Will double all frequencies

  // Terminal count for Timer 4 PWM
  OCR4C = 255;
}
void setup() {
  // put your setup code here, to run once:
  pwm613configure(PWM94k);
  pwm91011configure(PWM62k);
  pwm5configure(PWM62k);
}

void loop() {
  // put your main code here, to run repeatedly:

}
