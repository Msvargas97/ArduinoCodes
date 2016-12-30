void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Serial.println(F("Internal Temperature Sensor"));
}

void loop()
{
  // Show the temperature in degrees Celcius.
  Serial.println(GetTemp(),1);
  delay(1000);
}

double GetTemp(void)
{
  //ADC Multiplexer Selection Register
 ADMUX = 0;
 ADMUX |= (1 << REFS1);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
 ADMUX |= (1 << REFS0);  //Internal 2.56V Voltage Reference with external capacitor on AREF pin
 ADMUX &= ~(1 << MUX4);  //Temperature Sensor - 100111
 ADMUX &= ~(1 << MUX3);  //Temperature Sensor - 100111
 ADMUX |= (1 << MUX2);  //Temperature Sensor - 100111
 ADMUX |= (1 << MUX1);  //Temperature Sensor - 100111
 ADMUX |= (1 << MUX0);  //Temperature Sensor - 100111

 //ADC Control and Status Register A
 ADCSRA = 0;
 ADCSRA |= (1 << ADEN);  //Enable the ADC
ADCSRA |= ((1 << ADPS1) | (1<<ADPS0));  //ADC Prescaler - 8 (12MHz -> 1.5MHz)
 //ADC Control and Status Register B
 ADCSRB = 0;
 ADCSRB |= (1 << MUX5);  //Temperature Sensor - 100111
  ADCSRA |= (1 << ADSC);  //Start temperature conversion
  while (bitRead(ADCSRA, ADSC));  //Wait for conversion to finish
  uint8_t low  = ADCL;
  uint8_t high = ADCH;
  int temperature = (high << 8) | low;  //Result is in kelvin
  temperature -= 283;
  return temperature;
}
